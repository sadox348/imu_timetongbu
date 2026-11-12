# EIS（电子防抖）现状调研报告

- **日期**: 2026-11-12
- **平台**: Goertek sun / QCS8750 (sm8750), arch=apollo, chipcode pakala
- **基目录** (`$P`): `qc_private/LE.PRODUCT.11.0/apps_proc/src/vendor/qcom/proprietary`
- **信息来源**: camx / chi-cdk / camx-api 源码检索（路径均相对 `$P`）

## 总览

```
gyro/IMU (SSC/ADSP)                    tuning (eis17_sw.xml)
      │                                      │
      ▼                                      ▼
 NCS 数据通道 ──► ChiEISV2/V3 节点 (SW, 源码在仓库) ──► 闭源算法库 com.qti.eisv3.so
 (QSEE FastRPC)    camx/src/swl/gyro_features/    │ (dlopen, blob 不在仓库)
                                                   ▼
                                     warp grid / perspective 矩阵 (vendor tag)
                                                   │
                          ┌────────────────────────┴───────────────────────┐
                          ▼ realtime (IPE/OPE margin crop)                 ▼ video/offline
                    IPE/OPE 硬件裁剪                        EVA_MCTF / evawarp (aDSP 像素级 warp)
                                                              → IPE LMC 合成输出
```

## 1. 算法库集成

| 组件 | 位置 | 说明 |
|---|---|---|
| EIS 算法 C 接口头 | `camx-api/camx-lib/isalgo/api/eis_interface.h` | `eis_initialize` / `eis_process` / `eis_get_stabilization_margin_ex` 等 |
| **闭源算法库** | `com.qti.eisv2.so` / `com.qti.eisv3.so` | dlopen 动态加载；**blob 实体不在本仓库**（binary_sun 亦无），随发行包交付 |
| 动态加载点 | `camx/src/swl/gyro_features/eisv3/camxchinodeeisv3.cpp:55,909-923` | `pEIS3LibName="com.qti.eisv3"`；`LibGetAddr` 绑定 ~20 个函数指针；加载失败 sigabort |
| EISv2 同构 | `camx/src/swl/gyro_features/eisv2/camxchinodeeisv2.cpp:56,836-848` | `pEIS2LibName="com.qti.eisv2"` |
| GyroRNN | `camx/src/swl/gyro_features/gyrornn/camxchinodegyrornn.cpp:43,548-578` | 复用 com.qti.eisv3 库做 gyro RNN 序列处理 |
| 镜像打包 | `chi-cdk/configs/product.mk:62-70,286-287` | 节点模块 + 算法 so 均列入 vendor 模块清单 |

## 2. 节点实现（源码在仓库）

`camx/src/swl/gyro_features/` 下：

- **eisv2/** — ChiEISV2Node，preview 实时防抖
- **eisv3/** — ChiEISV3Node（~11000 行主实现）：look-ahead 帧延迟 + IFOVC/DIS/AdaptiveEIS + VideoSAT
- **gme/** — GME 全局运动估计（帧匹配，DIS/低光补充）；`wrapper/qtigmewrapper/` 为 QTI GME 算法 wrapper
- **gyrornn/** — gyro 序列管理（ChiGyroRNNSequenceInfo ×3）
- **utils/** — 公共工具（`com.qti.node.eisutils`，被 v2/v3 整库引入）
- **eisv3/oemcfg/** — `EISOEMConfig`：OEM 定制点（见 §6）

Vendor tag 注册：`camxchinodeeisv3.h:50-112`（`EISV3PerspectiveGridTransformTagId`、`EISV3DISMVStatsTagId` 等）。

## 3. 矩阵变换

- 变换函数在闭源库内，节点侧只绑指针（`camxchinodeeisv3.cpp:986-1006`）：
  - `eis_utility_calc_alignment_matrix`
  - `eis_utility_convert_output_stabilize_to_single_grid`（stabilize 输出 → IPE grid）
  - `eis_utility_convert_to_window_regions_deployment`
- 结果下发通道（vendor tag，`camxchinodeeisv3.cpp:1790-1843, 2194-2202`）：
  - ICA In Perspective Transform / Grid out2in / Grid lookahead
  - `EISV3PerspectiveGridTransform`
- chromatix 版本映射：sun → **EIS 1.7.0**（`eis_camx_utility_v1_7_0_to_is_chromatix_info`，CHROMATIX_VER_170_PLATFORMS）

## 4. 图像处理执行链路

| 路径 | warp 执行者 | 依据 |
|---|---|---|
| realtime preview | IPE/OPE margin crop | `camx/src/hwl/ope/camxopenode.cpp:2394-2432`（`UsecasePreviewPathEISCorrection`） |
| video / offline | **EVA (aDSP)** 像素级 warp | `camx/src/swl/evawarp/camxchinodeevawarp.cpp`；拓扑 `chi-cdk/oem/qcom/topology/apollo/.../UsecaseZSL/pipelines/FSOfflineVideoReprocess.xml`（EVA_MCTF → IPE LMC2） |

典型拓扑（`chi-cdk/oem/qcom/topology/apollo/usecase-components/usecases/`）：
- `UsecaseVideoEIS3PreviewEIS2/` — eisv2 + eisv3 挂同管线 + IPE_EIS2/3_MCTF
- `UsecaseVideo4kEIS3PreviewEIS2/`
- `camxMultiCameraEditableBokehEIS.xml:677-716` — eisv2/eisv3/OFE/EVA_MCTF/GME 共存

## 5. gyro/IMU 数据输入（与 IMU 工作交汇点）

1. `SetGyroDependency()`（`camxchinodeeisv3.cpp:3320-3365`）：`eis_get_time_intervals` 得 gyro 时间窗 → `ChiNCSDataRequest`（**QtimerNanoToQtimerTicks**，与 imu_qmi_recorder 同一时基域）→ NCS 异步取数，fence 挂依赖
2. `FillGyroData()`（`camxchinodeeisv3.cpp:4402-4495`）：`ChiIterateData` 逐样本填 `gyro_data_t.samples[]` → `eis_process`
3. OIS 数据同路（`SetOisDependency` / `FillOisData`，`camxchinodeeisv3.cpp:3406+`）
4. 底层实现：`camx/src/core/ncs/`（SSC/QMI 连接、QSEE 接口、direct channel、stub 回放、OIS 通道）
   - ⚠️ **已知问题**：此 NCS QSEE FastRPC 通道在 LE 平台验证过不出数（需走 QMI/sscrpcd）；采样率 clamp 补丁在 `camxncsintfqsee.cpp:549`
5. 调试注入：`/vendor/etc/camera/ifhgyro.csv`（IFH 伪 gyro 回放，`chi-cdk/api/isp/chieisdefs.h:114`）

## 6. 效果与开关配置

### 6.1 tuning：`eis17_sw.xml`（EIS 1.7.0，XSD `chi-cdk/api/chromatix/XSD/isp/eis_1_7_0.xsd`）

实例：`chi-cdk/oem/qcom/tuning/{sm8750_qtech_imx766, sm8750_shinetech_sc5a0xs_1, sm8750_shinetech_sc5a0xs_2}/Scenario.Default/XML/SW_CONTROL/eis17_sw.xml`
（三者参数值一致，双摄共用一套 EIS 参数，未按 sensor 差异化）

| 分区 | 关键参数 | 作用 |
|---|---|---|
| top | `requested_total_margins_x/y_*`（fhd/4k/8k/preview × 30/60/120） | margin = 防抖强度 |
| | `future_buffer_size_*` | look-ahead 帧延迟 |
| | `gyro_frequency`(400*)/`ois_frequency`(500)/acc/mag/ifovc/gravity_frequency | 采样频率（*我们本地已改 832→400，仍高于 ICM42607 的 200Hz 上限，靠 NCS clamp 兜底） |
| general | `minimal_total_margin`(0.2)、`focal_length`(1408)、noise floor | 基础参数 |
| timing | `gyro_offset`(-1072)、`ois_offset`(-345) 等 | 传感器-图像时间戳对齐偏移（防抖相位） |
| blur_masking | enable(0)、exposure_time_th、pan_threshold | 弱光/运动模糊降强度 |
| ois | data_conversion、filter、center 系列 | OIS 联动 |
| dis | enable(1)、confidence_threshold(75) | 数字防抖（低光 gyro） |
| z_lens_pos | `lut_z_lens_pos_tab`(16点)、fovc_mode(1) | zoom FOVC |
| selfie | face_distortion_prevention 系列 | 前置防畸变 |
| horizon_leveling | enable(0)、max_roll_leveling_deg(8) | 地平线校正 |
| adaptive_behavior | handheld/panning_boost、scene LUT、super_steady | 自适应强度 |

### 6.2 使能链

```
ANDROID_CONTROL_VIDEO_STABILIZATION_MODE (HAL)
  → ExtensionModule::UpdateEISMode()  chi-cdk/core/chiframework/chxextensionmodule.cpp:15938
  → UsecaseSelector::IsVideoEISV2/V3Enabled  chi-cdk/core/chiusecase/chxusecaseutils.cpp:1216-1284
  → feature2 graph selector  chi-cdk/oem/qcom/feature2/chifeature2graphselector/sm8750/chifeature2graphselector.cpp
      （按 EISMode/ImageStabilizationMode 选 EIS2/EIS3/NoEIS 拓扑与剪枝）
```

- StaticSettings：`EISV2Enable` / `EISV3Enable`（override settings 注入；本平台 `camxoverridesettings.txt` 目前**无 EIS 条目**）
- 低内存自动禁用（DeviceMemoryCapabilityLow）
- 节点级 override：`camxchinodeeisv3.cpp:226-249`（isOISDisabled / isDISDisabled / isIfovcEnabled 等）

## 7. OEM / 3rd-party 接入点

| 接入点 | 位置 | 可定制内容 |
|---|---|---|
| EISv3 算法库 | `pEIS3LibName`（同名替换 .so） | 整套算法 |
| **EISOEMConfig** | `camx/src/swl/gyro_features/eisv3/oemcfg/camxchieisoemcfg.cpp` | AdaptiveEIS 强度（`enableAdaptiveEISViaOEMConfig`）、VideoSAT 使能/帧延迟、EIS strength factor 输出 |
| GME wrapper | `camx/src/swl/gyro_features/gme/wrapper/qtigmewrapper/` | GME 实现 |
| usecase 选择 | `chi-cdk/oem/qcom/chiusecase/common/chxusecaseselector.cpp:623-641` | EIS variant 剪枝 |
| tuning | 各 sensor 目录 eis17_sw.xml | 效果参数（最直接抓手） |

## 8. 关键结论

1. **算法本体闭源**（com.qti.eisv3.so 不在仓库），效果调整主要抓手是 tuning（eis17_sw.xml）与 OEM hook
2. **效果三要素**：闭源算法库（不可改）、tuning 参数（margin/adaptive/timing offset 可调）、**gyro 数据能否进入 NCS 通道**——后者是本平台已知短板（NCS QSEE 在 LE 不出数，须走 QMI/sscrpcd；IMU 实测 200Hz 上限）
3. 双摄（IMX766 + SC5A0XS×2）共用同一套 EIS 参数，如需差异化需单独调
4. gyro 时间窗与 imu_qmi_recorder 同用 Qtimer ticks 域，时间戳对齐工作可直接复用
5. 调试捷径：`/vendor/etc/camera/ifhgyro.csv` 回放已录 gyro 即可验证 EIS 效果，不依赖真实 NCS 通路
