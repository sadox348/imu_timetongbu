# IMU / Camera SOF QTimer 时间戳对齐报告

**平台**: Goertek QCS8750 板 (Yocto machine `sun`, chipset pakala, LE Yocto, kernel 6.6)
**日期**: 2026-10-23
**目的**: EIS/OIS 前置 —— 验证 IMU (SSC/QMI 通道) 与相机帧 SOF 在同一 qtimer 时间轴上对齐

## 结论 (TL;DR)

| 验证项 | 标准 | 实测 | 结果 |
|---|---|---|---|
| IMU 样本率 | ≥ 预期 95% | 200.0 Hz (gyro/accel 各 6965 样本 / 34.89s) | ✅ |
| 时间戳单调性 | 单调递增无断流 | gyro/accel 均单调, dt_p99=5.017ms, 抖动 <10µs | ✅ |
| 到达延迟 | P99 < 50ms | gyro P99 = 3.74ms, accel P99 = 3.59ms | ✅ |
| SOF 帧数 | ≥ 100 帧 | 892 帧 (30s, 实际 fps = 30.00) | ✅ |
| 同轴覆盖 | 每帧窗口内 IMU 样本 | ±16.7ms 窗口内 gyro 样本 min=6 / med=7, 零覆盖帧 = 0/892 | ✅ |
| 晃动信号 | IMU 记录到晃动 | 静止 0.006 rad/s → 峰值 1.856 rad/s | ✅ |

**全部通过。** QMI/sscrpcd 通道的 IMU 与相机 SOF qtimer 时间戳在同一时间轴,可用于 EIS/OIS 帧级对齐。

## 数据采集方式

- **IMU**: `/data/imu_qmi_recorder -gyro_rate=200 -accel_rate=200 -duration=35 -out=/data/imu_35s.csv`
  (sensors-ship 自研测试 daemon, 走 QMI/sscrpcd (qsh) 通道, 即 sensors HAL 同款路径)
- **相机**: `gst-launch-1.0 -e qtiqmmfsrc camera=0 ! video/x-raw,width=1280,height=720,framerate=30/1 ! fakesink`
- **SOF 记录**: qmmf-server 日志 `camxsensor.cpp:2118 NotifyCSLMessage() Sensor[0] requestID: N, QTimer: SOFTimestamp: T` 行 (CAMX INFO 级)
- 采集窗口 35s, 期间手持板子做前后/左右/旋转晃动

## 关键技术事实

### 1. 时间戳单位陷阱: event_ts 是 tick 不是 ns

recorder CSV 的 `event_ts` 字段是 SSC 事件的**原始 19.2MHz qtimer tick**, 不是 ns。换算: `ns = tick × 52.0833`。`arrival_ts` 字段才是 ns (cntvct_el0 / 52.0833 后的 ns 值)。两字段单位不一致是刻意保留 (保留 SSC 原始精度), 分析时必须换算。CSV 表头已改为 `event_ts_ticks` 以明示。

### 2. logInfoMask 只有 setprop 可靠

camx static setting `logInfoMask` 控制所有 CAMX_LOG_INFO 输出, 默认 0 (全抑制)。
- 通过 `/vendor/etc/camera/camxoverridesettings.txt` 写入 `logInfoMask=0xFFFFFFFFFFFFFFFF` **在本板上未生效** (qmmf 重启后 INFO 日志仍被抑制)
- 通过 `setprop persist.vendor.camera.logInfoMask 18446744073709551615` **生效可靠** (该 setting 的 SetpropKey 路径)
- 因此 `QTimer: SOFTimestamp` INFO 日志 (camxsensor.cpp:2118) 的采集前提是先 setprop。meta-goertek 源文件中该行建议移除或加注释说明仅 setprop 生效

### 3. Qtimer / Mono 双时间戳链路稳定

camxcslhwinternal.cpp 把每帧 SOF 拆成两条 CSLMessage: Qtimer (timestampType=0) 与 Mono (timestampType=1)。实测 892 帧的 Qtimer−Mono 偏移: 中位 **3.743458s**, std 2.5ms —— 与 boot 时长一致, 全程稳定。EIS 应使用 Qtimer 值 (与 IMU 同域)。

### 4. SOF 日志解析格式

每帧两行可用日志 (取自 logcat, tag `CamX`):
```
... camxpipeline.cpp:7720 ... frameMessage:requestID=N, ... timestampType=0, Timestamp=<qtimer_sof>
... camxsensor.cpp:2118 ... Sensor[0] requestID: N, QTimer: SOFTimestamp: <qtimer_sof>, exposure start time: <qtimer_exp_start>
```
892 帧 SOF Qtimer 值与 exposure start 值均可从日志逐帧恢复。

## 数据质量明细

### IMU (QMI 通道)
```
[gyro]  N=6965 span=34.89s mean_dt=5009µs dt_p99=5017µs monotonic ✓ lat_med=1.50ms lat_p99=3.74ms
[accel] N=6965 span=34.89s mean_dt=5009µs dt_p99=5017µs monotonic ✓ lat_med=1.42ms lat_p99=3.59ms
```
- 200Hz 请求, 实际 199.6Hz (5009µs 均值) — 99.8% 达标率
- 到达延迟中位 1.5ms: SSC 批量投递 + QMI 传输开销, 事件时间戳本身是采样时刻, 不受投递延迟影响

### 相机 SOF
```
892 帧, span 29.73s, fps=30.00
frame dt: mean=33.37ms p1=33.33ms p99=33.33ms jitter_std=1.116ms
```
- 30fps 稳定, 帧间隔抖动 ~1ms (SOF 到达处理开销)

### 对齐覆盖
```
每帧 SOF ±16.7ms 窗口内 gyro 样本数: min=6 / med=7 / max=7, 零覆盖帧 = 0/892
```
- IMU 200Hz + 30fps → 每帧窗口理论 ~6.7 个样本, 实测中位 7 个, 无一帧掉出窗口
- 静止段 |gyro| ≈ 0.006 rad/s (噪声底), 晃动峰值 1.856 rad/s — 晃动动作清晰可辨

## 遗留项

1. **recorder 退出断言崩溃已修** (本次分析时发现): 原因是 main 返回时 sensor_stream 析构, QMI 接收线程仍持有捕获 `this` 的回调 lambda, 迟到回调锁已销毁 mutex 触发 robust-mutex 断言。修复: 退出前 `register_cb(suid, nullptr,...)` 注销回调 + `_running` 标志拦截迟到回调 (数据不受影响, 之前即逐行 flush)。
2. recorder 调试 fprintf 已删除; CSV 表头已更正为 `event_ts_ticks`。
3. 待提交: imu_qmi_recorder 源码 + Makefile.am; sensors-ship 另有 4 文件 registry 变更未提交 (需确认归属)。
4. 后续 (已暂停): GME/EIS NCSSensorDataSample 兼容输出接口。

## 复现步骤

```bash
# 1. 打开 camx INFO 日志 (setprop 才可靠)
adb shell setprop persist.vendor.camera.logInfoMask 18446744073709551615
adb shell killall qmmf-server   # 自动重生, 启动时读取 prop

# 2. 联采 (IMU 35s + 相机 30s + SOF 日志)
adb shell "logcat -c; (logcat -f /data/cam_35s.log -s ChiX:* CamX:* &); \
  /data/imu_qmi_recorder -gyro_rate=200 -accel_rate=200 -duration=35 -out=/data/imu_35s.csv & \
  sleep 3; timeout 30 gst-launch-1.0 -e qtiqmmfsrc camera=0 ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! fakesink; sleep 3; killall logcat"

# 3. 分析
adb pull /data/imu_35s.csv /data/cam_35s.log .
python3 align_report.py   # 注意 event_ts × 52.0833 → ns
```
