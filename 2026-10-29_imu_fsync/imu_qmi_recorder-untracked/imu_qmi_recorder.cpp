/** =============================================================================
 *  @file imu_qmi_recorder.cpp
 *
 *  [CQ8750s] IMU raw accel+gyro recorder over the QMI/sscrpcd (SEE) channel,
 *  for Camera-SOF / IMU qtimer timestamp alignment (EIS/OIS bring-up).
 *
 *  CAMX NCS QSEE1 FastRPC reverse-callback path delivers no data on this LE
 *  platform (P0 verdict 2026-10-20); this tool uses the verified QMI path
 *  (same as ssc_drva_test / sensors HAL) instead.
 *
 *  Usage: -gyro_rate=<hz> -accel_rate=<hz> -duration=<seconds> -out=<file>
 *  Output CSV per sample:
 *    <gyro|accel>,x,y,z,event_ts_ticks,arrival_ts_qns
 *  where event_ts_ticks is the raw SSC event timestamp in 19.2MHz qtimer
 *  ticks (multiply by 52.0833 for ns) and arrival_ts_qns is the local
 *  qtimer reading in ns (sensors_timeutil cntvct_el0) at callback time.
 *
 *  @copyright Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *  All rights reserved.
 *  Confidential and Proprietary - Qualcomm Technologies, Inc.
 *  ===========================================================================
 */

#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include "sns_client.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_std.pb.h"

#include "google/protobuf/io/zero_copy_stream_impl_lite.h"

#include "qsh_interface.h"
#include "sensors_log.h"
#include "suid_lookup.h"
#include "sensors_timeutil.h"

using namespace std;

/* ------------------------------------------------------------------------ */
/* qtimer helpers (same clock domain as SSC events and Camera SOF)          */
/* ------------------------------------------------------------------------ */

static uint64_t qtimer_now_ns()
{
    return sensors_timeutil::get_instance().qtimer_get_time_ns();
}

/* ------------------------------------------------------------------------ */
/* SUID lookup (pattern copied from ssc_drva_test locate class)             */
/* ------------------------------------------------------------------------ */

class suid_locator
{
public:
    sensor_uid lookup(const string& datatype)
    {
        /* fresh lookup per datatype — the suid lookup callback only
         * carries the datatype string, not the request context */
        _lookup_done = false;
        _suid = {0, 0};

        suid_lookup lookup(
            [this](const auto& datatype, const auto& suids)
            {
                this->on_suids_available(datatype, suids);
            });

        lookup.request_suid(datatype);

        sns_logi("waiting for suid lookup: %s", datatype.c_str());
        fprintf(stderr, "[imu_rec] waiting for suid lookup: %s\n",
                datatype.c_str());
        auto then = chrono::system_clock::now() + 5s;

        unique_lock<mutex> lk(_m);
        while (!_lookup_done) {
            if (_cv.wait_until(lk, then) == cv_status::timeout) {
                sns_loge("suid not found for %s", datatype.c_str());
                break;
            }
        }
        return _suid;
    }

private:
    void on_suids_available(const string& datatype,
                            const vector<sensor_uid>& suids)
    {
        sns_logi("%u sensor(s) available for datatype %s",
                 (unsigned)suids.size(), datatype.c_str());
        if (!suids.empty()) {
            _suid = suids[0];
            sns_logi("datatype %s suid = [%" PRIx64 " %" PRIx64 "]",
                     datatype.c_str(), _suid.high, _suid.low);
            fprintf(stderr, "[imu_rec] datatype %s suid = [%" PRIx64 " %" PRIx64 "]\n",
                    datatype.c_str(), _suid.high, _suid.low);
        }
        unique_lock<mutex> lk(_m);
        _lookup_done = true;
        _cv.notify_one();
    }

    string _datatype;
    sensor_uid _suid = {0, 0};
    bool _lookup_done = false;
    mutex _m;
    condition_variable _cv;
};

/**
 * DRVA-style override: force QMI even where glink qsh is available.
 * Strong definition overrides the weak one in qsh_interface_util.cpp.
 */
bool is_qsh_glink_supported()
{
    sns_logd("override default is_qsh_glink_supported");
    return false;
}

/* ------------------------------------------------------------------------ */
/* sensor stream: one qsh QMI connection + subscription per sensor          */
/* ------------------------------------------------------------------------ */

class sensor_stream
{
public:
    sensor_stream(const string& datatype, float rate_hz)
        : _datatype(datatype), _rate_hz(rate_hz) {}

    bool start(qsh_interface* qsh, sensor_uid suid)
    {
        _qsh = qsh;
        _qsh->register_cb(suid, nullptr, nullptr,
            [this](const uint8_t* msg, int msg_len, uint64_t ts)
            { this->handle_event(msg, msg_len, ts); });

        string encoded = create_config_request(suid);
        /* jumbo=true, same as USTA data_stream.cpp / see_workhorse paths */
        _qsh->send_request(suid, true, encoded);
        sns_logi("%s: subscribed, rate %f Hz", _datatype.c_str(), _rate_hz);
        return true;
    }

    uint64_t sample_count() const { return _count; }
    /* both streams write the same CSV: share one ofstream and one mutex */
    void set_output(shared_ptr<ofstream> out, shared_ptr<mutex> mtx)
        { _out = out; _out_mtx_shared = mtx; }
    bool has_output() const { return _out && _out->is_open() && _out->good(); }

    /* unregister the event callback before the stream objects are destroyed:
     * the QMI receive thread may still deliver a message afterwards, and its
     * lambda captures `this` — a late callback then locks a destroyed mutex
     * and trips the robust-mutex assertion in pthread_mutex_lock */
    void stop(qsh_interface* qsh, sensor_uid suid)
    {
        lock_guard<mutex> lk(_count_mtx);
        _running = false;
        qsh->register_cb(suid, nullptr, nullptr, nullptr);
    }

private:
    string create_config_request(sensor_uid suid)
    {
        sns_client_request_msg req;
        sns_std_sensor_config cfg;

        cfg.set_sample_rate(_rate_hz);
        string cfg_encoded;
        cfg.SerializeToString(&cfg_encoded);

        req.mutable_suid()->set_suid_high(suid.high);
        req.mutable_suid()->set_suid_low(suid.low);
        req.set_msg_id(SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_CONFIG);
        req.mutable_susp_config()->set_client_proc_type(
            SNS_STD_CLIENT_PROCESSOR_APSS);
        req.mutable_susp_config()->set_delivery_type(
            SNS_CLIENT_DELIVERY_NO_WAKEUP);
        /* HAL always sets a batch period (qsh_sensor.cpp activate path);
         * without it the SSC rejects the request with NOT_AVAILABLE. */
        req.mutable_request()->mutable_batching()->set_batch_period(0);
        req.mutable_request()->set_payload(cfg_encoded);

        string encoded;
        req.SerializeToString(&encoded);
        return encoded;
    }

    void handle_event(const uint8_t* data, int size, uint64_t /*time_stamp*/)
    {
        {
            lock_guard<mutex> lk(_count_mtx);
            if (!_running) {
                return;
            }
        }
        sns_client_event_msg event_msg;
        if (!event_msg.ParseFromArray(data, size)) {
            fprintf(stderr, "[imu_rec] %s: parse event_msg FAILED size=%d\n",
                    _datatype.c_str(), size);
            return;
        }

        for (int i = 0; i < event_msg.events_size(); i++) {
            const auto& pb_event = event_msg.events(i);
            if (pb_event.msg_id() == SNS_STD_MSGID_SNS_STD_ERROR_EVENT) {
                sns_std_error_event err_event;
                if (err_event.ParseFromString(pb_event.payload())) {
                    fprintf(stderr,
                            "[imu_rec] %s: ERROR event code=%d\n",
                            _datatype.c_str(), (int)err_event.error());
                }
            }
            if (pb_event.msg_id() != SNS_STD_SENSOR_MSGID_SNS_STD_SENSOR_EVENT) {
                continue;
            }

            sns_std_sensor_event sensor_event;
            if (!sensor_event.ParseFromString(pb_event.payload())) {
                sns_loge("%s: failed to parse sns_std_sensor_event",
                         _datatype.c_str());
                continue;
            }

            uint64_t event_ts = (uint64_t)pb_event.timestamp();
            uint64_t arrival_ts = qtimer_now_ns();

            lock_guard<mutex> lk(*_out_mtx_shared);
            if (*_out) {
                (*_out) << _datatype << ','
                     << sensor_event.data(0) << ','
                     << sensor_event.data(1) << ','
                     << sensor_event.data(2) << ','
                     << event_ts << ','
                     << arrival_ts << '\n';
                _out->flush();
            }
            _count++;
        }
    }

    string _datatype;
    float _rate_hz;
    qsh_interface* _qsh = nullptr;

    shared_ptr<ofstream> _out;
    shared_ptr<mutex> _out_mtx_shared;
    uint64_t _count = 0;
    mutex _count_mtx;
    bool _running = true;
};

/* ------------------------------------------------------------------------ */
/* main                                                                      */
/* ------------------------------------------------------------------------ */

static void usage(const char* prog)
{
    cout << "Usage: " << prog
         << " -gyro_rate=<hz> -accel_rate=<hz> -duration=<sec> -out=<file>"
         << endl;
}

int main(int argc, char** argv)
{
    float gyro_rate = 200.0f;
    float accel_rate = 200.0f;
    double duration = 30.0;
    const char* out_path = "/data/imu_qmi.csv";

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "-gyro_rate=", 11) == 0) {
            gyro_rate = atof(argv[i] + 11);
        } else if (strncmp(argv[i], "-accel_rate=", 12) == 0) {
            accel_rate = atof(argv[i] + 12);
        } else if (strncmp(argv[i], "-duration=", 10) == 0) {
            duration = atof(argv[i] + 10);
        } else if (strncmp(argv[i], "-out=", 5) == 0) {
            out_path = argv[i] + 5;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    cout << "imu_qmi_recorder: gyro=" << gyro_rate << "Hz accel="
         << accel_rate << "Hz duration=" << duration << "s out=" << out_path
         << endl;

    /* one shared ofstream + one mutex for both streams: two independent
     * append streams on the same file caused interleaved/corrupted CSV lines */
    auto out = make_shared<ofstream>(out_path);
    if (!out || !out->is_open()) {
        sns_loge("cannot open output file %s", out_path);
        return 1;
    }
    *out << "sensor,x,y,z,event_ts_ticks,arrival_ts_qns\n";

    suid_locator locator;

    sensor_uid gyro_suid = locator.lookup("gyro");
    if (gyro_suid.low == 0 && gyro_suid.high == 0) {
        sns_loge("gyro SUID not found, aborting");
        return 1;
    }
    sensor_uid accel_suid = locator.lookup("accel");
    if (accel_suid.low == 0 && accel_suid.high == 0) {
        sns_loge("accel SUID not found, aborting");
        return 1;
    }

    /* single shared QMI connection, like the HAL / ssc_drva_test pattern */
    qsh_interface* qsh = nullptr;
    {
        qsh_conn_config conn_config;
        try {
            qsh = qsh_interface::create(QSH_QMI, conn_config);
        } catch (const exception& e) {
            sns_loge("failed to create qsh_interface conn: %s", e.what());
            return 1;
        }
        if (qsh == nullptr) {
            sns_loge("qsh_interface::create(QSH_QMI) returned null");
            return 1;
        }
    }

    sensor_stream gyro_stream("gyro", gyro_rate);
    sensor_stream accel_stream("accel", accel_rate);
    auto out_mtx = make_shared<mutex>();
    gyro_stream.set_output(out, out_mtx);
    accel_stream.set_output(out, out_mtx);
    if (!gyro_stream.has_output() || !accel_stream.has_output()) {
        sns_loge("cannot open stream output files");
        return 1;
    }

    if (!gyro_stream.start(qsh, gyro_suid) || !accel_stream.start(qsh, accel_suid)) {
        sns_loge("failed to start sensor streams");
        return 1;
    }

    auto t_end = chrono::steady_clock::now()
                 + chrono::duration_cast<chrono::steady_clock::duration>(
                        chrono::duration<double>(duration));
    while (chrono::steady_clock::now() < t_end) {
        this_thread::sleep_for(1s);
    }

    /* stop streams before teardown: unregister callbacks so the QMI receive
     * thread can no longer call into streams that are about to be destroyed */
    gyro_stream.stop(qsh, gyro_suid);
    accel_stream.stop(qsh, accel_suid);
    delete qsh;

    cout << "done: gyro samples=" << gyro_stream.sample_count()
         << " accel samples=" << accel_stream.sample_count() << endl;
    cout << "output: " << out_path << endl;
    return 0;
}
