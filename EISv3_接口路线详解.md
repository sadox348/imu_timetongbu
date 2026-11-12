# EISv3 数据接口路线详解：SetGyroDependency() → FillGyroData() → eis_process() → Warp Metadata 输出

- **日期**: 2026-11-12
- **核心文件** (`$E`): `camx/src/swl/gyro_features/eisv3/camxchinodeeisv3.cpp`（~11000 行，所有行号均指此文件，除非另注明）
- **配套头文件**: 同目录 `camxchinodeeisv3.h`（vendor tag 注册 :50-112）；闭源算法 C 接口 `camx-api/camx-lib/isalgo/api/eis_interface.h`
- **前置阅读**: 同目录 `EIS_调研报告.md`（全局架构）

---

## 0. 全局时序图（先看这个）

```
应用下发 request N
   │
   ▼
[1] ProcessRequest(N)                    :7352     entry, sequenceNumber 决定走哪段
   │  sequenceId == 1
   ▼
[2] ProcessRequestSequenceIdOne          :7153
   │   └─ SetNCSDependency(N)            :3243     对每个 MFHDR 曝光:
   │       ├─ GetFrameInfo                :3192→   从算法拿本帧曝光时间(µs)
   │       ├─ GetTimeInterval            :3192    m_eisGetTimeInterval() → 算出
   │       │                                       gyro/ois/acc/mag/gravity 时间窗
   │       ├─ SetGyroDependency          :3315    ★建 SW sync fence + ChiNCSDataRequest
   │       │                                       (Qtimer ticks 时间窗), pGetData 异步订阅
   │       ├─ SetOisDependency           :3394    同路(OIS, µs 直传)
   │       ├─ SetAccelDependency / SetMagDependency / SetGravityDependency  同路
   │       └─ fence 全部挂进 pDependency->pChiFences[]   → 序号推进到 2
   │       (NCS service 不可用 → byPassAlgoExecution, 直接跳 seq 2, EIS 退化为默认 grid)
   ▼  ←←← 所有 fence signal 后, CHI 再次回调 ProcessRequest(N), sequenceId==2
[3] ProcessRequestSequenceIdTwo           :7200
   │   └─ ExecuteAlgo(N)                  :5100+
   │       ├─ 组装 is_input_t eisInputStab          :5100-5440
   │       │    ├─ FillGyroData(N, &input.gyro_data, &ti.gyro)   :4403 ★从 NCS 取回样本
   │       │    │    (ChiFetchData → pGetData → ChiIterateData 循环 → pPutData 释放)
   │       │    ├─ FillOisData / FillAccelData / FillGravityData / FillMagData   同构
   │       │    └─ (IFH 调试模式: ChiEISUtils::FillPseudoData 从 ifhgyro.csv 注入)
   │       ├─ m_eisProcess(m_pHEIS3Handles, &eisInputStab, pEis3OutputStab)  :6127 ★算法执行
   │       │    → is_output_t: stabilization_transform[3x3 透视矩阵], distortion_correction_grid,
   │       │      margin, has_output ...
   │       └─ GPU 部署时: m_eisCamXUtilityConvertStabilizeToGrid()   :6159
   │            (矩阵 → IPE 可用 grid, 并把 matrix 标志位关闭表示已转换)
   │   └─ UpdatePerspectiveAndGridTransform(N, ...)                  :8805
   │        ├─ FillPerspectiveAndGridTransform(N, ...)  从 is_output_t 抽取矩阵+grid
   │        └─ UpdatePerspectiveAndGridMetaData(N, ...)  :8831
   │             tagList[0]=EISV3PerspectiveGridTransformTagId →
   │             g_ChiNodeInterface.pSetMetadata(&metadataInfo)  ★写入 request N+m_lookahead 的
   │                                                        dynamic metadata (不是当前帧!)
   │   └─ pProcessMetadataDone(N)                                     :7266  立即放行 preview
   │   └─ 设置 future dependency: properties[0]=EISV3PerspectiveGridTransformTagId,
   │      offsets[0]=m_lookahead, negate=TRUE            :7298 → seq 推进到 3
   ▼  ←←← 等 request (N+m_lookahead) 的该 tag metadata 就绪后, 回调 seq==3
[4] ProcessRequestSequenceIdThree         :7321
   └─ PublishICAMatrixAndGrid(N, ...)     :8865
        └─ ChiNodeUtils::GetDataList(N + m_lookahead,
                EISV3PerspectiveGridTransformTagId, ChiMetadataDynamic)   :8898
            取出为未来帧算好的矩阵 → 转写为当前帧 N 的 ICA 系列 vendor tag 输出:
            ICAInPerspectiveTransform / ICAInGridOut2InTransform /
            ICAReferenceParams / EIS3ApproxCropWRTInput / EISOutCropFOV
   ▼
下游硬件节点 (IPE/OPE margin crop, 或 EVA warp) 读这些 tag 执行几何变换 → 防抖图像
```

**核心设计：两段式异步 + look-ahead。** 算法永远在"当前帧"执行，但结果写给"未来帧"（`N + m_lookahead`）；等未来帧真正到达时（seq 3），再把它取回作为该帧的最终 warp 元数据。`m_lookahead = DefaultEISV3FrameDelay`（:7983），算法需要看到未来几帧的 gyro 才能做前向预测（防抖本质上是预测手持运动趋势）。

---

## 1. 入口：ProcessRequest 与 sequenceId 状态机

```cpp
CDKResult ChiEISV3Node::ProcessRequest(CHINODEPROCESSREQUESTINFO* pProcessRequestInfo)  // :7352
{
    INT32 sequenceNumber = pDependencyInfo->processSequenceId;   // :7357
    UINT64 requestId    = pProcessRequestInfo->frameNum;         // :7358
```

`processSequenceId` 是 CHI 框架的重入机制：**同一个 request 会被 ProcessRequest 调用多次**，每次依赖满足后推进序号并重新调度。EISv3 用了三段：

| seq | 触发条件 | 干什么 |
|---|---|---|
| 1 | request 首次到达 | 建立所有 NCS 数据依赖（fence） |
| 2 | 全部 fence signal（数据到位） | 执行算法 + 写 matrix metadata + 设 future dependency |
| 3 | future metadata dependency 满足 | 把矩阵转写为当前帧 ICA 输出 tag |

这是理解整个路线的钥匙：**"数据依赖"与"metadata 依赖"分两个阶段解锁**。

---

## 2. SetNCSDependency（编排者，:3243-3311）

```cpp
CDKResult ChiEISV3Node::SetNCSDependency(CHINODEPROCESSREQUESTINFO* pProcessRequestInfo, UINT32 cameraIdIndex)
{
    for (UINT32 numExposure = 0; numExposure < m_numMFHDRExposureToProcess; numExposure++)
    {
        is_frame_info_t* pFrameInfo = ...GetFrameInfo(...);      // 从算法拿本帧曝光时间(µs)
        is_time_intervals_t timeInterval;
        ...
        GetTimeInterval(numExposure, pFrameInfo, &timeInterval); // :3192
        // ↑ 调闭源算法 m_eisGetTimeInterval(m_pHEIS3Handles, m_frame_times, ...)
        //   输入 = 各 MFHDR 曝光的起止时间, 输出 = 每个 sensor 需要的取数窗口

        CDKResult result = SetGyroDependency(pProcessRequestInfo, &timeInterval.gyro, numExposure);
        if (result == SUCCESS) { ... fence 挂入 pChiFences[chiFenceCount++] ... }
        SetOisDependency(...);        // :3394  OIS 线圈位置数据
        SetAccelDependency(...);       // 加速度(运动状态分类)
        SetMagDependency(...);         // 磁力计(方向)
        SetGravityDependency(...);     // 重力向量
    }
}
```

**为什么 MFHDR 循环**：多帧 HDR 时每个 exposure 有独立曝光窗口，gyro 窗口各不相同，`is_time_intervals_t` 里是按 exposure 的数组。`ValidateEISTimeInterval()`（:3238 附近）会检查窗口合法性（tStart < tEnd、不越界到未来）。

时间单位约定（**极易踩坑点**）：
- 算法接口层：**微秒** `MicroToNano` 处可见
- gyro/gravity/mag（SSC 域）：**Qtimer ticks** —— `QtimerNanoToQtimerTicks()`（32768kHz 时基）
- ois/accel（I2C 直读域）：**纳秒**直传，不做 ticks 转换
- 这与 imu_qmi_recorder 使用同一 Qtimer 域 —— 时间戳对齐工作因此可直接对接

---

## 3. SetGyroDependency（★建依赖，:3315-3389）

```cpp
CDKResult ChiEISV3Node::SetGyroDependency(
    CHINODEPROCESSREQUESTINFO* pProcessRequestInfo,
    is_time_interval_t*        pTimeInterval,
    UINT32                      numExposure)
{
    CDKResult      result = CDKResultSuccess;
    ChiNCSDataRequest dataRequest = {};
    CHIDATACALLBACKINFO dataCallbackInfo;
    CHIFENCEHANDLE hFence = NULL;
    static const CHAR* pChiFenceName = "ChiInternalFence_EISV3_Gyro";

    // (1) 创建 SW sync fence —— 一个可被 NCS 数据到位后手动 signal 的软件栅栏
    ChxFence *pFence = Chx fence 创建(ChiFenceTypeSWSync);
    hFence = reinterpret_cast<CHIFENCEHANDLE>(pFence);
    ...
    // (2) 组装 NCS 取数请求: 指明 sensor 类型 + 时间窗
    dataRequest.windowRequest.tStart = QtimerNanoToQtimerTicks(MicroToNano(pTimeInterval->first_ts));
    dataRequest.windowRequest.tEnd   = QtimerNanoToQtimerTicks(MicroToNano(pTimeInterval->last_ts));
    dataRequest.pfenceCallback        = hFence;           // 数据到 → signal 这个 fence
    dataRequest.pDataDone             = ...;              // NCS 层回调
    ...
    // (3) 发起异步订阅
    result = g_ChiNodeInterface.pGetData(m_hChiSession, GetDataSourceForGyro(), &dataRequest, NULL);
    if (result == CDKResultSuccess)
    {
        pProcessRequestInfo->pDependency->pChiFences[chiFenceCount++] = hFence;  // 挂依赖
    }
    else
    {
        ... 释放 fence ...   // 失败必须回收, 否则 fence 泄漏把整条管线挂死
    }
}
```

逐点注释：

1. **SW sync fence 而非硬件 fence**：gyro 数据不在硬件信号线上，到位与否只能由软件（NCS 层收到 QSEE/QMI 数据后）signal，所以用 `ChiFenceTypeSWSync`。
2. **窗口请求而非流式订阅**：EISv3 按帧精准拉取 `[tStart, tEnd]` 的样本 —— 算法只要覆盖曝光窗口的数据，不要多余数据（省带宽也省算法耗时）。
3. **失败路径**：`pGetData` 失败时若不释放 fence，seq 2 永远不会被触发 —— 上游 frame 会卡死，这是排查 "preview 卡住" 时值得打印的位置。
4. **`GetDataSourceForGyro()`** 返回 NCS 数据源句柄（区分 gyro/ois/accel/mag/gravity channel）。底层经 `camx/src/core/ncs/` 的 QSEE FastRPC 通道下发到 SSC。
   ⚠️ **本平台已知问题**：NCS QSEE FastRPC 通道在 LE 上验证不出数（IMU 工作结论），且 ICM42607 上限 200Hz < tuning 请求频率 —— 若 seq 2 永远不被触发、日志反复打印 `m_gyroNCSServiceAvailable` 相关 warning（:7186），第一怀疑对象就是这条通道。调试可用 `/vendor/etc/camera/ifhgyro.csv`（IFH 伪 gyro）绕过真实 NCS。

---

## 4. FillGyroData（★取数据，:4403-4495）

fence signal 后 seq 2 里 `ExecuteAlgo` 组装输入时调用：

```cpp
CDKResult ChiEISV3Node::FillGyroData(
    UINT64              requestId,
    is_sensor_data_t*   pGyroData,          // → is_input_t.gyro_data
    is_time_interval_t* pTimeInterval)
{
    ChiNCSDataRequest dataRequest = {};
    dataRequest.requestType = ChiFetchData;               // ★取回模式(区别于订阅模式)
    dataRequest.windowRequest.tStart/tEnd = 同 SetGyroDependency 的 ticks 窗口;
    dataRequest.chiDataCallback = ...;
    ...
    result = g_ChiNodeInterface.pGetData(m_hChiSession, GetDataSourceForGyro(), &dataRequest, NULL);
    // 返回 CHIDATAHANDLE + dataSize (clamped to GYRO_SAMPLES_BUF_SIZE)

    UINT32 numElements = 0;
    if (CDKResultSuccess == result)
    {
        ChiNCSGyroData* pNCSGyroData = ...;
        // 迭代 NCS 返回的每个样本块
        while (ChiIterateData(&iter))                       // 逐样本循环
        {
            pGyroData->samples[numElements].data[0] = pNCSGyroData->x;   // 3轴
            pGyroData->samples[numElements].data[1] = pNCSGyroData->y;
            pGyroData->samples[numElements].data[2] = pNCSGyroData->z;
            pGyroData->samples[numElements].data[3] = 0;    // 温度/保留位
            pGyroData->samples[numElements].ts =            // ★时间戳域转换
                NanoToMicro(QtimerTicksToQtimerNano(pNCSGyroData->timestamp));
            numElements++;
        }
        pGyroData->num_elements = numElements;
        g_ChiNodeInterface.pPutData(m_hChiSession, ..., &dataRequest);  // 归还缓冲
    }
}
```

逐点注释：

1. **两阶段 pGetData**：SetGyroDependency 用订阅+fence；FillGyroData 用 `ChiFetchData` 主动拉取。fence 只表示"数据到了"，真正样本在此取回。
2. **样本结构**：`is_sensor_data_t.samples[i]` = `{data[4], ts}`，`num_elements` 上限 `GYRO_SAMPLES_BUF_SIZE`（200Hz × 窗口时长内样本数上限）。
3. **时间戳双向换算**：请求时 `Nano→Ticks`，取回时 `Ticks→Nano→Micro` —— 算法内部统一微秒域。**做相机-IMU 时间戳对齐时，这就是两边交汇的转换点**，与 imu_qmi_recorder 的 Qtimer tick 输出可直接对表。
4. OIS/accel/mag/gravity 各有同构 `FillXxxData()`（OIS 在 :3464+ 等），代码模式完全一致，只是数据源与时间域（OIS 用 µs 直传）不同。

**IFH 调试旁路**（:5358 附近）：`CSLMode::CSLIFHEnabled` 时不走 NCS，改 `ChiEISUtils::FillPseudoData()` 从 `/vendor/etc/camera/ifhgyro.csv` 回放录制好的 gyro —— 验证算法/下游 warp 链路时不依赖真实 NCS 通路，本平台尤其有用。

---

## 5. eis_process（★算法执行，:6127）

`ExecuteProcess()`（:5100-5440 输入组装，:6127 调用）：

```cpp
// (1) 输入组装 —— 全部是栈上局部缓冲, 无堆分配(性能路径)
is_input_t eisInputStab = {};
input_gyro_data_t gyroData[GYRO_SAMPLES_BUF_SIZE];   // 3个 sensor 各自样本缓冲
...
eisInputStab.frame_times = m_frame_times;            // 曝光时间(µs), GetFrameInfo 缓存
eisInputStab.gyro_data   = &gyroData;                // FillGyroData 填充
eisInputStab.ois_data / accel_data / gravity_data / mag_data 同路
eisInputStab.pchromatix_info = m_pEISChromatixInfo;  // eis17_sw.xml 解析结果
eisInputStab.ifovc_x/y = 从 dynamic metadata 按 cameraId 读;
...
FillGyroData(requestId, &eisInputStab.gyro_data, &timeInterval.gyro);   // :5358

// (2) 执行
BOOL isResult = m_eisProcess(m_pHEIS3Handles, &eisInputStab, pEis3OutputStab);   // :6127
//     m_eisProcess = dlsym 绑定 (LoadLib, :909-923, :948-1009)
//     库名: "com.qti.eisv3.so" (pEIS3LibName, :55) —— blob 不在仓库

// (3) GPU 部署的矩阵→grid 转换
if ((DEP_TYPE_GPU_POST == ...) && (DIS grid 使能) && isResult)
{
    m_eisCamXUtilityConvertStabilizeToGrid(                            // :6159
        &pEis3OutputStab->distortion_correction_grid[0],
        &pEis3OutputStab->stabilization_transform[0],
        m_pScratchBuffer, ScratchBufferSize,
        &pEis3OutputStab->stabilization_transform[0].grid);
    // 矩阵已经被转进 grid, 关闭矩阵标志避免下游重复使用
}
m_eisUtilsGetEISApproxCrop(...);   // 近似裁剪量(供下游 crop 决策)
```

**输出 `is_output_t` 关键字段**（语义上）：
- `stabilization_transform[]` — 每个输出的 **3×3 透视变换矩阵**（防抖核心结果：本帧应如何 warp 才能抵消手持运动）
- `distortion_correction_grid[]` — 畸变校正 grid（与矩阵成对）
- `margin` / `EISApproxCrop` — 建议裁剪 margin（防抖必然牺牲 FOV）
- `has_output` — 本次是否产出有效结果

失败恢复：`m_currentAlgoOutputIndex` 双缓冲乒乓（:7211/7228），失败时回退 index 防止覆盖上一帧好结果。

---

## 6. Warp Metadata 输出（两跳发布）

### 6.1 第一跳：写给未来帧（seq 2 内）

```cpp
UpdatePerspectiveAndGridTransform(requestId, cameraIdIndex, pAlgoResult)   // :8805
{
    EISV3PerspectiveGridTransforms* pTrans = CHI_CALLOC(...);   // 矩阵+grid 打包结构
    FillPerspectiveAndGridTransform(requestId, pAlgoResult, pTrans);  // 从 is_output_t 抽取
    UpdatePerspectiveAndGridMetaData(requestId, cameraIdIndex, pTrans);  // :8831
}
UpdatePerspectiveAndGridMetaData(...)
{
    CHIMETADATAINFO metadataInfo;
    tagList[0] = m_vendorTagId.EISV3PerspectiveGridTransformTagId;   // ★关键 vendor tag
    tagData[0].pData     = pTrans;
    tagData[0].requestId = requestId;         // 注意: 这是刚执行算法的 request 号
    g_ChiNodeInterface.pSetMetadata(&metadataInfo);                  // 写 dynamic metadata
}
```

同时（:7292-7308）设 future dependency：

```cpp
pDependencyInfo->properties[0] = EISV3PerspectiveGridTransformTagId;
pDependencyInfo->offsets[0]    = m_lookahead;      // 依赖 "requestId+m_lookahead 的这个 tag"
pDependencyInfo->negate[0]      = TRUE;             // negate: tag 尚不存在就算满足(创建型依赖)
pDependencyInfo->processSequenceId = 3;
```

### 6.2 第二跳：未来帧到达时取回并转写（seq 3, :7321 → PublishICAMatrixAndGrid :8865-8919）

```cpp
PublishICAMatrixAndGrid(requestId, isEndOfStream, isEISv3Disabled, cameraIdIndex)
{
    if (EISv3 disabled)  → FillDefaultPerspectiveGridTransform();           // 单位矩阵兜底
    else if (stop recording / endOfStream)
        → 从 m_pEndOfStreamOutputArray[requestId % QueueDepth] 拷贝缓存
    else
    {
        ChiNodeUtils::GetDataList(requestId + m_lookahead,                  // :8898
            m_vendorTagId.EISV3PerspectiveGridTransformTagId,
            ChiMetadataDynamic, &pDataList);
        // ↑ 取出当初写给未来帧的矩阵
        ... 转写为本帧输出 tags:
        ICAInPerspectiveTransformTagId      // ICA 部署: 透视矩阵
        ICAInGridOut2InTransformTagId       // ICA 部署: out2in grid
        ICAReferenceParamsTagId / EIS3ApproxCropWRTInput / EISOutCropFOVTagId
    }
}
```

**发布 tag 集合按部署类型二选一**（`QueryMetadataPublishList` :8238-8282）：

| 部署 | 发布的 tag | 下游消费者 |
|---|---|---|
| `DEP_TYPE_GPU_POST`（GPU） | 仅 `ICAInGridTransformLookAheadTagId` | GPU 后处理（矩阵已由 `eis_utility_convert_output_stabilize_to_single_grid` 转 grid） |
| ICA（默认, 本平台） | ICAInPerspective + GridOut2In + ReferenceParams + ApproxCrop + OutCropFOV | IPE/OPE 硬件（margin crop）或 EVA warp（视频） |

### 6.3 endOfStream 特殊处理（:7276-7291）

停止录像时应用设 endOfStream vendor tag；此处 `pForceDRQtoRemoveDependencies` 强制清依赖（look-ahead 依赖未来帧，但录像结束后未来帧不再来了），并把 `m_lastEIS3publishedRequest` 收敛到 `frameNum - m_lookahead`，避免最后一帧矩阵丢失。

---

## 7. 关键数据结构速查

| 结构 | 出处 | 作用 |
|---|---|---|
| `CHINODEPROCESSREQUESTINFO` | chi-cdk api | 每次节点处理的请求上下文；`pDependency` 携带 fence/tag 依赖与 processSequenceId |
| `ChiNCSDataRequest` | chi-cdk ncs api | NCS 取数请求：`requestType`(订阅/ChiFetchData) + `windowRequest`(tStart/tEnd, ticks) + fence 回调 |
| `is_time_intervals_t` | eis_interface.h 域 | 算法算出的各 sensor 取数窗口（gyro/ois/acc/mag/gravity 各一个 `is_time_interval_t`） |
| `is_sensor_data_t` | 同上 | `{samples[].data[4], samples[].ts(µs), num_elements}` |
| `is_input_t` | 同上 | 算法全量输入：frame_times + 各 sensor 数据 + chromatix + ifovc |
| `is_output_t` | 同上 | 算法输出：stabilization_transform 3×3 矩阵 + grid + margin + has_output |
| `EISV3PerspectiveGridTransforms` | camxchinodeeisv3.h | 第一跳 metadata 载荷（矩阵+grid 打包） |
| vendor tags | camxchinodeeisv3.h:50-112 | `EISV3PerspectiveGridTransformTagId`（内部接力）+ ICA 系列最终输出 |

---

## 8. 调试与实验建议（针对本平台）

1. **验证链路是否走到每一段**（CHI 日志组 `ChxLogGroupChiNode`，日志字样）：
   - seq 1 失败：`"EIS disabled for frameNum ... m_gyroNCSServiceAvailable"`（:7186）→ NCS 服务不可用
   - fence 不 signal：无 seq 2 日志，preview 卡死 → NCS QSEE 通道不出数（本平台已知）
   - seq 2 失败：`"EISv3 algo execution failed"`（:7225）→ dlopen/算法本身
2. **不依赖 NCS 验证后半段**：放 `ifhgyro.csv` 开 IFH 模式，可完整跑 FillPseudoData→eis_process→warp 输出。
3. **时间戳对齐实验**：FillGyroData 的 `QtimerTicksToQtimerNano(...)` 是与 imu_qmi_recorder（QMI 直采）可比对的同一时基；两边对表即可量化 sensor-图像相位（对应 tuning 的 `gyro_offset=-1072µs`）。
4. **改效果优先动 tuning**：`eis17_sw.xml` 的 margin/offset/adaptive 分区（算法闭源不可改；OEM hook 在 `eisv3/oemcfg/camxchieisoemcfg.cpp`）。
5. 采样率现实：tuning `gyro_frequency`（本地已改 400）仍高于 ICM42607 的 200Hz 上限，`camxncsintfqsee.cpp:549` 的 clamp 是实际生效值。
