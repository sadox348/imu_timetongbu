# IMU ODR 上限回退至 1000Hz — 构建与验证说明

日期: 2026-10-30
板子: Goertek QCS8750 (sun), ADSP 固件 pakala.adsp.prod

## 1. 改动内容

`LPAIDSP.HT.1.1/adsp_proc/ssc_drivers/icm4x6xx/src/hal/sns_icm4x6xx_hal.h`

```c
#else   /* !ICM4X6XX_ENABLE_SEE_RBX */
  #define ICM4X6XX_MAX_LOW_LATENCY_RATE ICM4X6XX_ODR_1000   /* 原为实验值 ODR_4000, 现回退 stock */
#endif
```

回退原因: 4000Hz 订阅与 720p@30 相机流并发时两次导致板子 USB 掉线(疑似
PM8012 供电跌落); 1000Hz 实测对齐精度 p50 247µs / p99 492µs, 已满足 ±0.5ms
目标。完整实验数据见同目录 patch 文件的 commit message。

注意: 工作区中其余未提交改动为 FSYNC 实验, 已留档于
`patches/2026-10-29_imu_fsync/0001-adsp-imu-42688-fsync.patch`, 与本次回退无关。

## 2. 编译(服务器上)

```bash
cd /data/magine_yan/qc_private/LPAIDSP.HT.1.1/adsp_proc/build/ms
python ./build_variant.py pakala.adsp.prod
```

产物(约 5-6 分钟):
`bin/pakala.adsp.prod/splitbins/signed/adsp.mdt + adsp.b00–b57`

## 3. 部署到板子

```bash
ADB="~/bin/adb -s e1bc454f"
SRC=/data/magine_yan/qc_private/LPAIDSP.HT.1.1/adsp_proc/build/ms/bin/pakala.adsp.prod/splitbins/signed

$ADB shell 'mount -o remount,rw /firmware'
$ADB push $SRC/adsp.mdt /firmware/image/adsp.mdt
$ADB shell 'mkdir -p /data/adsp_new'
for i in $(seq -w 0 57); do $ADB push $SRC/adsp.b$i /data/adsp_new/adsp.b$i; done
$ADB shell 'for i in $(seq -w 0 57); do cp /data/adsp_new/adsp.b$i /firmware/image/adsp.b$i; done; sync'
$ADB shell 'md5sum /firmware/image/adsp.mdt; md5sum /firmware/image/adsp.b00'
md5sum $SRC/adsp.mdt $SRC/adsp.b00   # 两边对比一致
$ADB shell 'mount -o remount,ro /firmware'
```

注意:
- adsp.b30 / adsp.b56 为 0 字节是正常的(ELF filesz=0 段), 旧固件备份中
  它们非零是因为段布局不同, 不要据此判断部署失败。
- 部署后必须**整板断电重启**(用户操作), 仅 `reboot` 不一定会重新加载 ADSP 镜像。

## 4. 重启后验证

### 4.1 ADSP 正常起来

```bash
$ADB shell 'cat /sys/class/remoteproc/remoteproc1/state'   # 期望: running
$ADB shell 'ps -A | grep sscrpcd'                          # 期望: sscrpcd 在跑
```

### 4.2 回退生效: >1000Hz 被拒绝, ≤1000Hz 正常

```bash
# 期望: 输出文件有样本, ~1000 行/秒 (实际 1006.7Hz, 时钟偏快 0.67%)
$ADB shell '/usr/bin/imu_qmi_recorder -gyro_rate=1000 -accel_rate=1000 -duration=5 -out=/tmp/v1000.csv 2>/dev/null | tail -1; wc -l /tmp/v1000.csv'

# 期望: 无输出文件 + SNS_RC_INVALID_VALUE (add_new_request 拒绝),
# stderr 无 "[imu_rec] waiting for suid lookup" 之后的报错, 直接 exit 1
#   —— 与 4000Hz 固件行为相反(那边会正常出数据)
$ADB shell '/usr/bin/imu_qmi_recorder -gyro_rate=2000 -accel_rate=2000 -duration=3 -out=/tmp/v2000.csv 2>/dev/null; echo E=$?; ls /tmp/v2000.csv 2>&1'
```

判断标准:
- 1000Hz: `gyro samples≈5000` (5s×1000), E=0
- 2000Hz: E=1, `/tmp/v2000.csv: No such file or directory` → **回退生效**

### 4.3 (可选) 相机并发回归

```bash
# 热身(第一次起流会 SIGSEGV, 属已知现象)
$ADB shell 'GST_PLUGIN_SYSTEM_PATH_1_0=/usr/lib/gstreamer-1.0 timeout 20 gst-launch-1.0 -e qtiqmmfsrc camera=0 eis=false ! video/x-raw,width=1280,height=720,framerate=30/1 ! fakesink >/dev/null 2>&1; echo G=$?'

# 并发录制 20s: 相机流 + 1000Hz IMU, 全程不掉线
$ADB shell 'nohup /usr/bin/imu_qmi_recorder -gyro_rate=1000 -accel_rate=1000 -duration=25 -out=/data/vsync.csv >/dev/null 2>&1 & sleep 2; GST_PLUGIN_SYSTEM_PATH_1_0=/usr/lib/gstreamer-1.0 timeout 20 gst-launch-1.0 -e qtiqmmfsrc camera=0 eis=false ! video/x-raw,width=1280,height=720,framerate=30/1 ! fakesink >/dev/null 2>&1; echo G=$?'
$ADB shell 'wc -l /data/vsync.csv'   # 期望 ≈ 50000+ 行 (gyro+accel 各 ~20000)
```

## 5. 回滚到 4000Hz 实验(如需要)

改 `sns_icm4x6xx_hal.h` 中该宏为 `ICM4X6XX_ODR_4000`(或 2000), 重复第 2-3 节。

## 6. 已知坑

- adb shell 必须是 root(`id` 显示 uid=0); 非 root 时 /data 不可写,
  recorder 会"无 stderr 直接 exit 1"。/tmp 始终可写(非 root 也能录到 /tmp)。
  root/非 root 由 /tmp/.adb.root 标志决定, 内容 `#ROOT###` 为 root。
  **不要执行 `adb root`**(历史上两次断连), 让用户自己操作或断电重启。
- 4000Hz 档实际输出 3911Hz(0b0110 ODR tick 偏慢 ~2.2%), 不是时钟问题。
- recorder 退出时 glibc robust-mutex assertion 崩溃是已知良性现象, 数据已落盘。
