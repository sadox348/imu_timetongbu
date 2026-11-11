# Makefile.am - Automake script for imu_qmi_recorder
# [CQ8750s] IMU raw accel+gyro recorder over QMI/sscrpcd for
# Camera-SOF / IMU qtimer timestamp alignment (EIS/OIS bring-up).

AM_CPPFLAGS = -Werror                       \
              -Wall                         \
              -Wno-unused-parameter         \
              -Wno-unused-variable          \
              -fexceptions                  \
              -DSNS_TARGET_LE_PLATFORM      \
              -I./                          \
              -I../utils                    \
              -I../../qsh                   \
              -I../../qsh/qsh_interface     \
              -I../../utils                 \
              -I../../sensors-log

cpp_sources = ./imu_qmi_recorder.cpp

requiredlibs = ../../qsh/libsnsapi.la                   \
               ../../sensors-log/libsensorslog.la       \
               ../../qsh/libqsh.la                      \
               ../../utils/libsnsutils.la               \
               -ldl                                     \
               -llog                                    \
               -lcutils                                 \
               $(PROTOBUF_LIBS)

bin_PROGRAMS =  imu_qmi_recorder
imu_qmi_recorder_SOURCES = $(cpp_sources)
imu_qmi_recorder_CC = @CC@
imu_qmi_recorder_CPPFLAGS = $(AM_CPPFLAGS)
imu_qmi_recorder_LDFLAGS = -shared -version-info 1:0:0
if USE_GLIB
 imu_qmi_recorder_CPPFLAGS += -DUSE_GLIB @GLIB_CFLAGS@
 imu_qmi_recorder_LDFLAGS += @GLIB_LIBS@
endif
imu_qmi_recorder_LDADD = $(requiredlibs)
