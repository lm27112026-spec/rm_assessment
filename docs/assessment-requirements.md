# RM 装甲板视觉考核项目技术需求与实现方案

## 1. 文档说明

本文档用于整理项目全部考核需求、当前代码基础、预计实现方案和验收标准，并作为后续开发、联调和答辩的执行依据。

需求状态定义如下：

- **已具备基础实现**：仓库中已有对应代码，但仍需使用真实设备和数据完成效果验收。
- **待实现**：仓库中尚无可交付实现，需要后续开发。
- **发挥项**：在基础题目完成后实施的增强功能。
- **TBD**：参数或资源尚未确定，必须通过数据集、设备资料或现场标定获得，不能直接照搬参考代码中的数值。

### 1.1 只读目录约束

`learning/` 是参考学习代码目录，整个开发周期内均为只读：

1. 不得修改、格式化、重命名或删除其中任何文件。
2. 不得在其中生成构建产物、日志、缓存、标定结果或训练输出。
3. 可阅读其中的接口设计和算法流程，但正式实现必须放在项目自己的功能目录中。
4. 提交前使用 `git status --short -- learning` 和 `git diff -- learning` 确认该目录无变更。

## 2. 项目目标与范围

项目最终需要形成一套可在 Windows 和 Ubuntu 上构建运行的 C++17 工程，覆盖摄像头取流、传统视觉装甲板识别、YOLO 装甲板识别、目标跟随、数字识别、电脑与 STM32 串口通信，以及可选的距离和三维姿态解算。

总体数据流预计为：

```text
摄像头/视频
    -> myCamera 统一取流
    -> 传统视觉或 YOLO 检测
    -> 装甲板候选框与角点
    -> 数字识别
    -> 目标选择与时序跟随
    -> 距离/位姿解算（发挥项）
    -> 串口封帧
    -> USB 转 TTL
    -> STM32
    -> OLED 显示
```

## 3. 当前项目基础

经仓库盘点，当前代码基础如下：

| 模块 | 当前状态 | 现有位置 | 说明 |
| --- | --- | --- | --- |
| 摄像头封装 | 已具备基础实现 | `src/MyCamera/myCamera.hpp`、`src/MyCamera/myCamera.cpp` | 公有接口仅包含构造、析构和 `read`；私有变量均以 `_` 结尾；支持无 OpenCV 时 fake 模式降级编译 |
| 摄像头示例 | 已具备 | `io/example.cpp` | 支持摄像头源参数和画面显示 |
| 传统视觉检测 | 已具备基础实现 | `src/MyArmorTraditional/detector.*`、`src/MyArmorTraditional/armor.hpp`、`src/MyArmorTraditional/img_tools.hpp` | 固定阈值二值化、灯条筛选、灯条配对、装甲板候选（`auto_aim` 命名空间） |
| 数字识别 | 已具备基础实现 | `src/MyArmorTraditional/detector.cpp`（整合在 `auto_aim::Detector` 内） | 通过 OpenCV DNN 加载 ONNX 分类模型，作为检测门控 |
| 目标跟随 | 已具备基础实现 | `tasks/detection_tracker.hpp`（YOLO demo 用） | 轻量卡尔曼跟踪器 `rm_assessment::DetectionTracker` |
| 综合演示 | 已具备基础实现 | `tasks/armor_demo.cpp` | 串联检测、分类与可视化 |
| 自动测试 | 已具备部分测试 | `tests/` | 覆盖摄像头接口、合成灯条检测、模型异常处理、跟踪状态转换和 YOLO 推理 |
| YOLO 检测 | 已具备基础实现 | `src/MyArmorYolo/yolov5.*`、`src/MyArmorYolo/yolov5_utils.*`、`models/yolov5/` | YOLOv5 OpenVINO 推理，`YOLOV5Detector` 类（命名空间 `rm_assessment::yolov5`），输出带颜色/数字分类的 `Detection` 结构；`learning/` 内相关 YOLO 代码仅作只读参考 |
| 上下位机通信 | 已具备基础实现 | `communication/`、`stm32/` | PC 端 `mySerial` 跨平台串口类 + `frame` 帧构造/解析（`0xAA + 12B payload + 0xBB`）；STM32 端 UART 环形缓冲 + 帧解析状态机 + SSD1306 OLED 显示 |
| 距离/姿态解算 | 已部分实现（发挥项） | `tasks/yolov5_pnp_demo.cpp` | YOLOv5 检测结合 `cv::solvePnP`（`SOLVEPNP_IPPE`）解算装甲板位姿，含距离输出、重投影校验与坐标轴 Y/P/R 绘制；`learning/tasks/auto_aim/solver.*` 可只读参考 |
| 父子级 CMake | 已具备基础实现（发挥项） | 根 `CMakeLists.txt` + `src/MyCamera/`、`src/MyArmorTraditional/`、`src/MyArmorYolo/`、`communication/`、`tasks/`、`tests/` 各子模块 `CMakeLists.txt` | 根工程通过 `add_subdirectory` 统一管理各模块；YOLO 通过 `YOLO_WITH_OPENVINO` 选项按需启用 |

“已具备基础实现”不等同于完成最终验收。传统视觉、数字模型、摄像头跨平台行为和跟踪鲁棒性仍需使用实验室装甲板与真实视频验证。

### 3.1 已冻结参数（2026-08-12）

以下参数已经过确认并冻结，后续开发和验收以此为准：

| 领域 | 参数 | 冻结值 | 说明 |
|---|---|---|---|
| **YOLOv5 模型** | 输入尺寸 | 640×640 | `yolov5.cpp:prepare_input()` |
| | 置信度阈值 | 0.7 | `YOLOV5Detector::confidence_threshold_` |
| | NMS 阈值 | 0.3 | `YOLOV5Detector::nms_threshold_` |
| | 输出格式 | 22 列 (score + 4 color + 8 corners + 9 digits) | `yolov5_utils.hpp` 常量 |
| | 模型来源 | 公开/预训练模型 | 需补充训练数据集和 License 说明 |
| | 推理后端 | OpenVINO (CPU) | `YOLO_WITH_OPENVINO` CMake 选项 |
| **数字识别** | 模型文件 | `learning/assets/tiny_resnet.onnx` | 9 类: one~five, sentry, outpost, base, not_armor |
| | 输入尺寸 | 32×32 灰度图 | `Detector::classify()` |
| | 类别映射 | 0-8 → `ArmorName`（one~five / sentry / outpost / base / not_armor） | `Detector::classify()` 内 softmax 取最大类 |
| **传统视觉** | 检测参数 | 灰度阈值 120 及几何阈值常量 | `detector.cpp` 内常量，无 `Params` 结构体 |
| | 跟踪参数 | max_match=120px, max_temp_lost=8 | `DetectionTracker::Params`（仅 YOLO demo） |
| **装甲板尺寸** | 灯条高度 | 56 mm | 与 `learning/tasks/auto_aim/solver.cpp` 一致 |
| | 小装甲板宽度 | 135 mm | 同上 |
| | 大装甲板宽度 | 230 mm | 同上 |
| **相机内参** | 当前 fallback | fx=1818.37, fy=1822.49, cx=751.06, cy=530.44 | `calibration/camera_params.yaml`，**需实际标定后替换**；标定程序见 `calibration/calibrate_camera.cpp` |
| **串口协议** | 波特率 | 115200 8N1 | `docs/protocol.md` v1.0 已冻结 |
| | 帧格式 | 14 字节: 0xAA + 12B LE payload + 0xBB | XOR 校验 (offset 0-10 → offset 11) |
| | 时序 | PC 端 20 Hz (50ms) | STM32 端 UART RX ISR → ring buffer → main loop parse |
| **STM32** | MCU | STM32F103C8T6 (Blue Pill) | `stm32/README.md` |
| | OLED | SSD1306 128×64 I2C (地址 0x3C) | PB6(SCL)/PB7(SDA), 3.3V |
| | 引脚 | PA9(TX), PA10(RX), PC13(LED) | 已烧录验证 |
| **验收阈值** | 传统视觉 FPS | ≥ 100 fps | 目标平台实际分辨率下 |
| | YOLO FPS | ≥ 30 fps | OpenVINO CPU 推理 |
| | 传统视觉 Recall | ≥ 85% | 实验室真实装甲板视频 |
| | 传统视觉 Precision | ≥ 80% | 同上 |
| | 数字识别准确率 | ≥ 85% (数字 1-5) | 固定验证集上统计 |
| | YOLO mAP@0.5 | ≥ 0.85 | 独立测试集评估 |
| | YOLO 跟踪 ID 跳变 | < 5 次/分钟 | 真实视频连续跟踪 |
| | 距离误差 | 1-3m 范围平均 < 10%, 最大 < 20% | PnP 解算 vs 实测距离 |
| | 姿态重投影误差 | < 5 px | 坐标轴方向需定性正确 |
| | 通信正确率 | ≥ 99.9% (1000 帧) | 错误帧注入后 5 帧内恢复同步 |
| **开发约束** | `learning/` 只读 | 任何提交不得包含 `learning/` 变更 | `git diff -- learning` 必须为空 |

### 3.2 仍需后续确认的 TBD

| 待确认项 | 影响 | 确认方式 |
|---|---|---|
| 相机内参（实际标定值） | PnP 距离/姿态精度 | 用实验室摄像头拍摄标定板，运行 `calibration/calibrate_camera.cpp`，替换 `calibration/camera_params.yaml` 中的 fallback 值 |
| YOLO 训练数据集来源和 License | 文档完整性 | 在模型说明中补充数据集来源、版本、许可证 |
| 字符编码 (MSVC C4819 警告) | 构建警告 | 统一源文件为 UTF-8 BOM 或调整 MSVC 编译选项 |

## 4. 功能需求总表

| 编号 | 功能需求 | 优先级 | 当前状态 | 完成判据 |
| --- | --- | --- | --- | --- |
| FR-01 | 封装跨 Windows/Ubuntu 的 `myCamera` 类 | 必做 | 已具备基础实现 | 两个平台均能构建并读取摄像头或视频帧 |
| FR-02 | `myCamera` 公有成员仅有构造函数、析构函数和 `read` | 必做 | 已满足 | 头文件接口检查不出现其他公有成员 |
| FR-03 | 所有私有成员变量名称以 `_` 结尾 | 必做 | 已满足 | 代码审查和静态检查通过 |
| FR-04 | 示例文件位于 `./io/example.cpp` | 必做 | 已满足 | 文件存在且可构建运行 |
| FR-05 | 使用 GitHub 管理代码 | 必做 | 已具备仓库 | 提交历史清晰，主分支可复现构建 |
| FR-06 | 传统视觉框选完整装甲板 | 必做 | 已具备基础实现 | 实验室装甲板视频中稳定输出完整框 |
| FR-07 | 传统视觉目标跟随 | 必做 | 已具备基础实现 | 短暂漏检时仍保持预测，重新出现后恢复跟踪 |
| FR-08 | 装甲板数字识别 | 必做 | 已具备基础实现 | 在固定验证集上数字 1-5 识别准确率 ≥ 85% |
| FR-09 | YOLO 系列装甲板框选 | 必做 | 已具备基础实现 | 模型能对视频逐帧输出装甲板检测框和置信度，mAP@0.5 ≥ 0.85 |
| FR-10 | YOLO 检测结果跟随 | 必做 | 已具备基础实现 | 检测结果接入统一跟踪器，输出连续目标轨迹，ID 跳变 < 5 次/分钟 |
| FR-11 | PC 与 STM32 通信链路 | 必做 | 已具备基础实现 | PC 发送 1000 帧正确率 ≥ 99.9%，STM32 正确解析并在 OLED 显示，错误帧注入后 5 帧内恢复同步 |
| FR-12 | 帧格式为 `0xAA + 数据 + 0xBB` | 必做 | 已具备基础实现 | 错帧被丢弃，合法帧能完整解析 |
| EX-01 | 题目2、3、4分别封装为独立模块并分类存放：`MyArmorTraditional`、`MyArmorYolo`、`communication`/`mySerial` | 发挥项 | 已具备基础实现 | 类职责清晰，模块间通过明确接口通信 |
| EX-02 | 使用父子级 CMake 管理各模块 | 发挥项 | 已具备基础实现 | 根工程一次配置即可构建各库、示例和测试 |
| EX-03 | 鲁棒识别旋转装甲板灯条并框选整块装甲板 | 发挥项 | 待增强 | 旋转、倾斜和一定模糊条件下仍能获得正确角点 |
| EX-04 | 解算装甲板与摄像头距离 | 发挥项 | 已部分实现 | `tasks/yolov5_pnp_demo.cpp` 已实现 `cv::solvePnP` 距离解算；需验证 1-3m 范围平均误差 < 10%、最大误差 < 20% |
| EX-05 | 解算装甲板三维姿态并绘制坐标系 | 发挥项 | 已部分实现 | `tasks/yolov5_pnp_demo.cpp` 已实现坐标轴投影与 Y/P/R 角度显示；需验证重投影误差 < 5px，坐标轴方向定性正确 |

## 5. 题目1：myCamera 跨平台封装

### 5.1 必须保持的接口约束

`myCamera` 的公有成员必须严格限制为：

```cpp
explicit myCamera(const std::string & source = "0");
~myCamera();
bool read(cv::Mat & img,
          std::chrono::steady_clock::time_point & timestamp);
```

不得为了方便增加 `open`、`close`、`isOpened`、参数设置函数或公有数据成员。初始化和资源释放分别由构造函数与析构函数负责；读取成功后由 `read` 同时输出图像和单调时钟时间戳。

### 5.2 预计实现方式

1. 使用 OpenCV `cv::VideoCapture` 统一封装摄像头编号、视频文件和网络流。
2. 将字符串形式的纯整数源解释为摄像头编号，其他字符串解释为文件路径或 URL。
3. Windows 优先使用可用的视频采集后端，Ubuntu 使用系统可用后端；平台差异仅留在 `.cpp` 内部。
4. 析构函数显式释放采集资源，避免设备句柄泄漏。
5. 私有变量保持 `_` 后缀规则，例如 `source_`、`capture_`、`opened_`。
6. 示例固定保留在 `io/example.cpp`，通过命令行参数选择视频源。

### 5.3 验收项目

- Windows：MSVC/CMake 构建成功，USB 摄像头连续取流，按 `q` 或 `Esc` 正常退出。
- Ubuntu：GCC/CMake 构建成功，摄像头设备或视频文件连续取流。
- 连续读取的时间戳单调递增；读取失败时返回 `false`，不输出无效帧。
- 使用头文件审查或编译期接口测试确认没有额外公有成员。

## 6. 题目2：OpenCV 传统视觉装甲板识别

### 6.1 功能分解

1. **灯条提取**：从 BGR 图像中提取高亮且符合敌方颜色特征的区域。
2. **灯条几何筛选**：按轮廓面积、长宽比、长度、方向角等条件剔除噪声。
3. **灯条配对**：按颜色、长度比、角度差、中心高度差、间距比例等条件组成装甲板。
4. **完整框选**：计算左右灯条构成的四角点、外接框和中心点。
5. **ROI 生成**：沿灯条方向扩展（1.125 倍灯条长度）得到装甲板 ROI，供分类使用。
6. **数字识别**：使用 OpenCV DNN 运行 ONNX 分类模型，输出类别（`ArmorName`）和置信度，并作为检测门控过滤误检。
7. **目标跟随**（YOLO demo）：根据中心距离选择同一目标，使用轻量卡尔曼滤波平滑并预测。
8. **结果显示**：绘制候选四边形、跟踪框、中心点、数字、置信度、状态和 FPS。

### 6.2 传统视觉算法流程

```text
BGR 图像
 -> 灰度化 + 固定阈值(120)二值化
 -> findContours
 -> minAreaRect 获取旋转矩形
 -> 灯条几何与颜色筛选
 -> 灯条两两配对
 -> 装甲板几何校验
 -> 沿灯条方向扩展得到 ROI，ONNX 分类
 -> 置信度>0.8 且非 not_armor 才保留
 -> 绘制结果
```

检测阈值以常量形式集中在 `detector.cpp`。真实验收前应采集不同距离、角度、曝光、背景和运动速度下的数据进行调参。

### 6.3 数字识别方案

识别已整合进 `auto_aim::Detector`：

1. 对灯条扩展得到的装甲板 ROI 做灰度化、尺寸统一和数值归一化。
2. 通过 OpenCV DNN 加载 ONNX 分类模型（`Detector` 构造时加载一次，路径可配）。
3. 对网络 logits 做数值稳定 softmax，取最大类别写入 `Armor::name`。
4. 置信度低于 0.8 或类别为 `not_armor` 时该候选被丢弃，避免误检。
5. 使用真实装甲板图像建立独立验证集，记录混淆矩阵、准确率和低置信度比例。

分类模型来源、类别定义、输入尺寸和最终精度目标均为 `TBD`，须以招新群数据或实际模型说明为准。

### 6.4 跟随方案

传统视觉 demo（`armor_demo`）直接输出每帧检测结果，不做时序跟踪。

YOLO demo（`yolov5_armor_demo`）使用 `rm_assessment::DetectionTracker`（`tasks/detection_tracker.hpp`），四维恒速卡尔曼跟踪框中心，状态包括：

- `lost`：无有效目标；
- `tracking`：目标已确认并持续更新；
- `temp_lost`：短暂漏检，使用预测框维持输出。

后续需要补充多目标选择策略，例如优先数字、距离画面中心、候选置信度和连续目标 ID，防止多个装甲板之间频繁跳变。

### 6.5 验收项目

- 静止装甲板：能框住左右灯条组成的完整装甲板，而非只框单根灯条。
- 运动装甲板：检测框连续，跟踪输出无明显跳变。
- 短暂遮挡：在设定丢失帧数内保留预测，超过阈值后正确进入 `lost`。
- 数字识别：在固定验证集上统计准确率，不以少量演示帧代替量化结果。
- 性能：在目标平台和分辨率下记录平均 FPS、P95 单帧耗时和漏检率；具体目标值为 `TBD`。

## 7. 题目3：YOLO 系列装甲板检测与跟随

### 7.1 资源准备

1. YOLO 模型明确选用 **YOLOv5**。
2. 已将 `learning/assets/yolov5.xml` 和 `learning/assets/yolov5.bin` 复制到项目正式模型路径 `models/yolov5/`，后续推理只读取 `models/yolov5/yolov5.xml` 与 `models/yolov5/yolov5.bin`，不直接依赖 `learning/`。
3. `learning/tasks/auto_aim/yolos/yolov5.*`、`learning/tasks/auto_aim/yolo.*` 和相关检测后处理代码仅作为只读参考，用于理解 OpenVINO 推理、letterbox、NMS、坐标还原和装甲板结果组装流程。
4. 仍需确认 YOLOv5 模型的类别表、输入尺寸、置信度阈值、NMS 阈值和标签语义；这些参数不能凭文件名推断，统一标记为 `TBD`。
5. 若后续补充视觉招新群数据集或自采数据，应单独记录数据来源、许可证、划分方式和评估结果。

模型版本和项目内模型路径已确定；数据集地址、类别表、输入尺寸、阈值配置和目标精度仍为 `TBD`。

### 7.2 软件设计

项目自有的 YOLOv5 模块位于 `src/MyArmorYolo/`，通过 `YOLO_WITH_OPENVINO` CMake 选项按需启用：

| 文件 | 说明 |
|---|---|
| `yolov5.hpp/.cpp` | `rm_assessment::yolov5::YOLOV5Detector` 类，构造接收模型路径与设备名，`detect(frame)` 返回 `std::vector<Detection>` |
| `yolov5_utils.hpp/.cpp` | `Detection` 结构定义（`box`、`confidence`、`class_id`、`corners[4]`、`color_id`、`digit_id`）、sigmoid、NMS 等后处理工具 |

`Detection` 结构包含装甲板特有字段：
```cpp
struct Detection {
    cv::Rect2f box;
    float confidence = 0.0F;
    int class_id = 0;
    std::array<cv::Point2f, 4> corners{};  // 装甲板四角点
    int color_id = 0;                       // 颜色分类
    int digit_id = 0;                       // 数字分类
};
```

模型输出格式：每行 22 列（score + 4 颜色 + 8 角点 + 9 数字），通过 `kScoreColumn`、`kColorStartCol`、`kDigitStartCol` 等常量索引。

### 7.3 验收项目

- 在独立测试集上报告 Precision、Recall、mAP 和各类别结果。
- 在真实视频上完成框选与连续跟随，记录漏检、误检和 ID 跳变。
- 框坐标在不同输入宽高比下能正确映射回原图。
- CPU 或目标设备上的平均 FPS 满足最终约定值。
- 模型缺失、格式不兼容或推理失败时返回明确错误，不导致未定义行为。
- 验收时确认程序实际加载的是 `models/yolov5/yolov5.xml`，而不是 `learning/assets/yolov5.xml`。

## 8. 题目4：电脑与 STM32 上下位机通信

### 8.1 硬件组成

- 电脑；
- STM32 套件，具体型号为 `TBD`；
- USB 转 TTL 模块；
- OLED 显示模块；
- 面包板和杜邦线；
- 共地连接。

基础接线原则：USB-TTL 的 TX 接 STM32 UART_RX，RX 接 UART_TX，GND 必须共地。电平必须按模块规格确认，默认不得假定所有模块均可承受 5 V。OLED 使用 I2C 或 SPI 取决于实验室模块，接口类型为 `TBD`。

### 8.2 串口帧协议

题目规定最小帧结构为：

```text
+--------+----------------------+--------+
| 0xAA   | 数据 payload         | 0xBB   |
+--------+----------------------+--------+
  帧头            数据             帧尾
```

数据区的字段、长度、字节序、浮点编码、波特率和发送周期尚未给定，全部标记为 `TBD`。推荐在正式协议中采用固定长度字段或增加长度与校验字段，但只有在不违反考核规定且与下位机同步确认后才能扩展。

建议的最小解析状态机：

1. `WAIT_HEADER`：丢弃字节直到收到 `0xAA`。
2. `READ_PAYLOAD`：按约定固定长度读取数据。
3. `WAIT_TAIL`：期望收到 `0xBB`；否则丢弃当前帧并重新同步。
4. `FRAME_READY`：解析数据、更新内部状态并刷新 OLED。

若数据区可能出现 `0xBB`，必须依靠固定长度、长度字段或转义机制确定边界，不能只搜索第一个帧尾。

### 8.3 上位机预计实现

定义 `mySerial` 或 `myCommunication` 类，负责：

- 跨平台打开和关闭串口；
- 配置波特率、数据位、停止位和校验位；
- 把视觉结果序列化为字节数组；
- 添加 `0xAA` 帧头和 `0xBB` 帧尾；
- 完整发送一帧并报告错误；
- 必要时接收 STM32 应答。

Windows 可使用 Win32 串口 API，Ubuntu 可使用 termios。平台相关代码应隐藏在实现文件中，业务层只使用统一接口。也可使用工程中已批准的成熟跨平台串口库，但需在 CMake 和文档中声明依赖。

### 8.4 下位机预计实现

定义 STM32 端接收模块，负责：

- UART 中断或 DMA 接收；
- 环形缓冲区缓存字节流；
- 帧头、固定长度数据和帧尾解析；
- 非法帧丢弃与重新同步；
- 将解析后的目标坐标、距离或状态刷新到 OLED；
- 统计正确帧、错误帧和超时，便于联调。

STM32 工程建议独立存放，不与主机 C++ 目标混合编译，但应在同一仓库中保留协议定义和说明。主机与单片机两端应共享一份协议字段表，防止字段顺序和单位不一致。

### 8.5 验收项目

- PC 连续发送至少 1000 帧，STM32 解析数量和内容正确。
- 断开或重新插入 USB-TTL 后，程序能报告错误并允许重新连接。
- 注入缺失帧头、错误帧尾和截断帧，STM32 不更新错误数据且能恢复同步。
- OLED 按约定频率显示最新有效数据，无明显闪烁或阻塞 UART 接收。
- 使用逻辑分析仪或串口调试工具留存 `AA ... BB` 的实测字节证据。

## 9. 发挥项实现方案

### 9.1 类封装与目录分类

项目按以下结构组织。题目1、2、3分别由 `MyCamera`、`MyArmorTraditional`、`MyArmorYolo` 管理；题目4保持在已独立建模的 `communication` 和 `stm32` 目录中：

```text
rm_assessment/
├── CMakeLists.txt
├── cmake/
│   └── helpers.cmake
├── run.ps1 / run.sh                 # 跨平台构建与运行脚本
├── io/
│   └── example.cpp                  # 题目1 示例（仅此文件保留在 io/）
├── src/
│   ├── MyCamera/
│   │   ├── CMakeLists.txt
│   │   └── myCamera.hpp/.cpp        # 题目1：myCamera 跨平台封装（支持无 OpenCV fake 模式）
│   ├── MyArmorTraditional/
│   │   ├── CMakeLists.txt
│   │   ├── armor.hpp
│   │   ├── detector.hpp/.cpp
│   │   └── img_tools.hpp           # 题目2：传统视觉 + 识别（auto_aim 命名空间）
│   └── MyArmorYolo/
│       ├── CMakeLists.txt
│       ├── yolov5.hpp/.cpp
│       └── yolov5_utils.hpp/.cpp    # 题目3：YOLOv5 OpenVINO 检测（YOLO_WITH_OPENVINO 启用）
├── communication/
│   ├── CMakeLists.txt
│   ├── frame.hpp/.cpp               # 帧构造/解析（0xAA + 12B payload + 0xBB）
│   ├── mySerial.hpp/.cpp            # 跨平台串口类（Win32/termios）
│   └── tests/
│       ├── frame_test.cpp           # 帧单元测试（10 用例）
│       └── loopback_test.cpp        # 端到端回环测试（1000 帧）
├── stm32/
│   ├── task4_serial.ioc             # CubeMX 工程文件
│   ├── Makefile                     # arm-none-eabi-gcc 命令行构建
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── frame_parser.h       # 帧解析状态机 API（WAIT_HEADER → READING_PAYLOAD → WAIT_FOOTER）
│   │   │   ├── uart_ring.h          # UART 环形缓冲区 API（ISR-safe, 256B）
│   │   │   ├── ssd1306.h            # SSD1306 OLED 文本驱动 API（I2C, 6×8 字体）
│   │   │   └── ...                  # HAL 配置、GPIO、I2C、USART、中断声明
│   │   └── Src/
│   │       ├── frame_parser.c       # 帧解析实现（固定 12B payload + XOR 校验）
│   │       ├── uart_ring.c          # 中断驱动环形缓冲实现
│   │       ├── ssd1306.c            # SSD1306 I2C 文本渲染与 page flush
│   │       ├── main.c               # 主循环：轮询解析 → 刷新 OLED → LED 心跳
│   │       └── ...                  # HAL 初始化与中断服务
│   └── README.md                    # 独立构建说明（不接入根 CMake）
├── tasks/
│   ├── CMakeLists.txt
│   ├── armor_demo.cpp               # 传统视觉综合演示
│   ├── yolov5_armor_demo.cpp        # YOLOv5 检测 + 跟随演示
│   ├── detection_tracker.hpp        # YOLO 演示用轻量卡尔曼跟踪器
│   ├── yolov5_pnp_demo.cpp          # YOLOv5 + PnP 位姿解算演示（发挥项 EX-04/05）
│   ├── serial_demo.cpp              # 视觉→串口集成演示
│   └── serial_loopback.cpp          # PC 端回环工具（硬件缺席保底）
├── tests/
│   ├── CMakeLists.txt
│   ├── myCamera_test.cpp            # 摄像头接口测试
│   ├── myCamera_interface_test.cpp  # 接口合规测试
│   ├── armor_vision_test.cpp        # 传统视觉测试
│   ├── yolov5_test.cpp              # YOLO 推理测试
│   └── output/                      # 测试输出目录
├── docs/
│   ├── assessment-requirements.md   # 本文档
│   └── protocol.md                  # 题目4 串口协议
├── models/
│   └── yolov5/                      # YOLOv5 模型文件（yolov5.xml + yolov5.bin）
└── learning/                        # 只读，不参与修改
```

各模块集中于 `src/` 下按题目2/题目3分类，题目4保持在 `communication/` 与 `stm32/` 中。发挥项 EX-04/05 的 PnP 位姿解算由 `tasks/yolov5_pnp_demo.cpp` 独立实现，复用 `YOLOV5Detector` 检测结果后调用 `cv::solvePnP`。演示程序和测试只依赖公开目标与头文件目录，不通过相对路径耦合内部布局。

STM32 固件工程通过 `stm32/Makefile` 支持 `arm-none-eabi-gcc` 命令行构建，`Core/Inc/` 中的 `frame_parser.h`、`uart_ring.h`、`ssd1306.h` 分别封装帧解析状态机、UART 环形缓冲和 OLED 文本驱动，与上位机 `communication/frame.hpp` 共享一致的 `#pragma pack(1)` payload 结构体定义。

### 9.2 父子级 CMake

根 `CMakeLists.txt` 只负责：

- 设置 C++ 标准和通用编译选项；
- 查找 OpenCV、推理后端和串口依赖；
- 调用 `add_subdirectory`；
- 开启测试。

各子目录通过自己的 `CMakeLists.txt` 定义库、公开头文件和依赖。当前形成 `mycamera`、`armor_vision`、`yolov5`、`communication` 等目标，再由 `tasks/` 中的演示程序组合链接。禁止使用全局 `include_directories` 污染所有目标。

### 9.3 旋转装甲板灯条鲁棒识别

传统视觉增强方向：

1. 使用 `minAreaRect` 保留灯条旋转角，而非依赖水平外接框。
2. 对灯条角度差、长度比、中心连线角、纵向偏移和间距比例做联合约束。
3. 以左右灯条的长边端点构造装甲板四角点，保证旋转时仍框选整块区域。
4. 对角点顺序统一为左上、右上、右下、左下，避免透视变换翻转。
5. 根据曝光和环境光考虑 HSV/亮度自适应、局部阈值或多阈值候选融合。
6. 跟踪阶段可用上一帧预测区域限制 ROI，但必须在目标丢失时恢复全图搜索。
7. 使用旋转台或带角速度标注的视频，按角度、速度、距离分别统计召回率。

### 9.4 距离解算

在装甲板平面与相机成像模型已知时，使用 `cv::solvePnP` 解算装甲板相对于摄像头的平移向量 `tvec`：

1. 标定摄像头，获得相机内参矩阵和畸变系数。
2. 按真实装甲板尺寸建立四个三维角点，坐标原点可设在装甲板中心。
3. 从检测器或 YOLO 关键点结果获得对应的四个二维角点。
4. 调用 `solvePnP` 得到 `rvec` 和 `tvec`。
5. 距离可定义为光心到目标中心的欧氏距离 `sqrt(x^2+y^2+z^2)`，也可按需求输出相机 Z 轴深度；文档和 OLED 必须明确单位和定义。
6. 使用 `projectPoints` 计算重投影误差，误差过大时拒绝本帧结果。

相机内参、畸变系数、装甲板长宽、距离定义和允许误差均为 `TBD`，必须通过标定和实际测量确定。

### 9.5 三维姿态与坐标系绘制

复用 PnP 输出的 `rvec` 和 `tvec`：

1. 在装甲板坐标系中定义原点及 X、Y、Z 三个轴端点。
2. 使用 `cv::projectPoints` 将坐标轴投影到图像。
3. 使用固定颜色绘制 X/Y/Z 轴，并在画面标注坐标系定义和距离。
4. 检查角点顺序、右手坐标系、相机坐标方向和长度单位，避免姿态镜像。
5. 使用不同 yaw、pitch 和 roll 的实拍姿态验证方向变化是否符合实际。

由于平面 PnP 可能存在多解和抖动，应结合重投影误差、上一帧姿态和适合平面目标的 PnP 方法筛选解，并对位姿做时序滤波。

## 10. 统一数据结构与模块接口

传统视觉和 YOLO 应向后续模块输出统一结果，建议至少包含：

```text
DetectionResult
- bounding_box：轴对齐显示框
- corners[4]：有序装甲板角点；若模型不提供则标记不可用
- center：目标中心
- color：红/蓝/未知
- digit/class_id：数字或类别
- confidence：检测或分类置信度
- timestamp：采集时间戳
- source：traditional/yolo
```

需要三维解算时，四个角点是必要输入。若 YOLO 模型只输出矩形框，框角点不一定等于装甲板物理角点，不能直接声称可获得高精度 PnP；应改用关键点模型、角点细化或传统灯条几何与 YOLO ROI 融合。

## 11. CMake 与跨平台构建计划

当前根 CMake 已使用 C++17，并按 OpenCV 是否存在决定是否构建视觉模块。工程已经按父子级 CMake 拆分：

1. 根 `CMakeLists.txt` 只设置标准、查找依赖、定义选项并调用 `add_subdirectory`。
2. `src/MyCamera/`、`src/MyArmorTraditional/`、`src/MyArmorYolo/`、`communication/`、`tasks/`、`tests/` 分别维护自己的 `CMakeLists.txt`。
3. 使用 `target_include_directories` 和 `target_link_libraries` 表达依赖。
4. YOLOv5 通过 `YOLO_WITH_OPENVINO` 选项按需启用，启用时自动查找 OpenVINO 并复制运行时 DLL。
5. 对 Windows/Ubuntu 的串口实现（`communication/mySerial.cpp`）使用 `#ifdef _WIN32` 条件编译。
6. 模型文件、配置文件通过运行参数传入，不把开发机绝对路径编译进程序。
7. 构建目录必须位于 `learning/` 之外，例如 `build/` 或 `out/build/`。

建议基础验证命令：

```bash
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Windows 和 Ubuntu 应在 GitHub Actions 或两台真实环境上分别执行构建与测试。涉及摄像头、串口和 OLED 的硬件测试不能只依赖 CI，需要单独保存现场验收记录。

### 11.1 Windows 运行时 DLL 搜索路径问题

> **已解决**：本项目已通过 `cmake/helpers.cmake` 的 `copy_opencv_runtime_dlls` 函数，在构建后自动把 OpenCV 与 MinGW 运行时 DLL 复制到各可执行文件同目录（应用于 `armor_demo`、`serial_demo`、`armor_vision_test`、`mycamera_test`）。以下为问题分析与历史方案说明。

**问题**：Windows 下构建成功后，直接双击或命令行运行可执行文件时，弹出系统错误对话框提示找不到 `opencv_world4100d.dll`（Debug 配置）或 `opencv_world4100.dll`（Release 配置），程序无法启动。这是 OpenCV 动态链接库不在 Windows DLL 搜索路径中导致的，属于本项目最常见的运行时错误之一。

**根因**：Windows 加载 DLL 时按以下顺序搜索：可执行文件所在目录 → 系统目录（`System32`）→ `PATH` 环境变量 → 当前工作目录。CMake 构建的可执行文件位于 `build/<config>/` 子目录中，而 OpenCV 的 DLL 位于 OpenCV 安装目录的 `bin/` 下，两者不在同一目录且 `PATH` 中通常不含该路径，因此运行时加载失败。

**推荐解决方案（按优先级）**：

1. **CMake 自动复制 DLL 到输出目录（推荐，一劳永逸）**：

   在根 `CMakeLists.txt` 或各子模块的 CMake 中，对每个生成的可执行目标追加生成后事件：

   ```cmake
   # 查找 OpenCV DLL 所在目录
   get_target_property(OpenCV_BIN_DIR_DEBUG
       opencv_world IMPORTED_LOCATION_DEBUG)
   get_filename_component(OpenCV_BIN_DIR_DEBUG
       "${OpenCV_BIN_DIR_DEBUG}" DIRECTORY)

   # 对目标自动复制 DLL（Debug 与 Release 分别处理）
   add_custom_command(TARGET my_target POST_BUILD
       COMMAND ${CMAKE_COMMAND} -E copy_if_different
           "$<$<CONFIG:Debug>:${OpenCV_BIN_DIR_DEBUG}/opencv_world4100d.dll>"
           "$<$<CONFIG:Debug>:$<TARGET_FILE_DIR:my_target>>"
       COMMAND ${CMAKE_COMMAND} -E copy_if_different
           "$<$<CONFIG:Release>:${OpenCV_BIN_DIR_RELEASE}/opencv_world4100.dll>"
           "$<$<CONFIG:Release>:$<TARGET_FILE_DIR:my_target>>"
   )
   ```

   或者使用更简洁的方式，通过 `OpenCVConfig.cmake` 自动定位：

   ```cmake
   if(WIN32 AND OpenCV_FOUND)
       foreach(target_name my_target_1 my_target_2)
           if(TARGET ${target_name})
               add_custom_command(TARGET ${target_name} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_if_different
                       "${OpenCV_DIR}/bin/${CMAKE_BUILD_TYPE}/*.dll"
                       "$<TARGET_FILE_DIR:${target_name}>"
                   COMMENT "Copying OpenCV DLLs to ${target_name} output dir"
               )
           endif()
       endforeach()
   endif()
   ```

2. **运行脚本设置 PATH（开发调试用）**：

   在 `run.ps1` / `run.sh` 启动脚本中临时追加 OpenCV DLL 目录到 `PATH`：

   ```powershell
   # run.ps1 (PowerShell)
   $OpenCV_BIN = "D:\opencv\build\x64\vc16\bin"
   $env:PATH = "$OpenCV_BIN;$env:PATH"
   .\build\Debug\armor_demo.exe
   ```

3. **IDE 内直接运行（VS Code / Visual Studio）**：

   - **VS Code**：在 `launch.json` 的 `"environment"` 中追加 `PATH`。
   - **Visual Studio**：项目属性 → 调试 → 环境，添加 `PATH=D:\opencv\build\x64\vc16\bin;%PATH%`。

4. **系统级安装 OpenCV 到 PATH**（不推荐）：将 OpenCV bin 目录永久添加到系统环境变量。缺点是与不同项目、不同 OpenCV 版本共存困难，且污染全局环境。

**验收要求**：任意开发者 clone 仓库并完成构建后，无需手动配置环境变量即可通过 `cmake --build` 和后续脚本直接运行可执行文件，不应再出现 `找不到 DLL` 错误。

**常见变体**：除 `opencv_world4100d.dll` 外，根据 OpenCV 版本和编译选项不同，可能遇到的 DLL 名称包括 `opencv_world480.dll`、`opencv_core4100.dll`、`opencv_imgproc4100.dll` 等。解决方案通用，只需调整 DLL 文件名或使用通配符复制。



## 12. 测试与验收计划

| 测试编号 | 对应需求 | 测试内容 | 通过标准 |
| --- | --- | --- | --- |
| T-01 | FR-01~FR-04 | 构建并检查 `myCamera` 接口、命名、取流和时间戳 | 两平台构建成功，接口与命名满足硬约束 |
| T-02 | FR-06 | 合成灯条与真实装甲板检测 | 合成测试通过，真实数据 Recall ≥ 85%, Precision ≥ 80% |
| T-03 | FR-07 | 匀速移动、快速移动和短时遮挡跟踪 | 状态转换正确，轨迹稳定，丢失后可恢复 |
| T-04 | FR-08 | 数字分类测试集 | 输出混淆矩阵和准确率，数字 1-5 ≥ 85% |
| T-05 | FR-09 | YOLO 独立测试集评估 | 输出 Precision、Recall、mAP，mAP@0.5 ≥ 0.85 |
| T-06 | FR-10 | YOLO 视频跟踪 | 跟踪连续，ID 跳变率 < 5 次/分钟 |
| T-07 | FR-11~FR-12 | 1000 帧串口循环和错误帧注入 | 合法帧正确率 ≥ 99.9%，错误帧不更新显示，5 帧内恢复同步 |
| T-08 | EX-02 | 全新构建目录配置和编译 | 根命令一次构建全部启用模块 |
| T-09 | EX-03 | 多角度、多转速旋转装甲板视频 | 完整框选召回率 ≥ 85% |
| T-10 | EX-04 | 多个已知距离测量 | 1-3m 范围平均误差 < 10%，最大误差 < 20% |
| T-11 | EX-05 | 已知姿态与坐标轴投影 | 轴方向正确，重投影误差 < 5px |
| T-12 | 全部 | `learning/` 只读门禁 | `git status --short -- learning` 无输出 |

每个算法测试应记录：数据集版本、参数文件、模型哈希、运行平台、图像分辨率、帧率、准确率指标和失败样例。不得只展示一段效果良好的视频作为全部验收证据。

## 13. GitHub 代码管理要求

1. 使用功能分支开发，例如 `feature/yolo-detector`、`feature/serial-link`、`feature/pose-solver`。
2. 每次提交只包含一个明确目的，避免同时提交算法修改、目录迁移和大批生成文件。
3. 提交模型时先确认文件大小和许可证；大模型建议使用 Git LFS 或发布附件。
4. `.gitignore` 应排除构建目录、IDE 文件、日志、标定临时文件和训练输出。
5. 每次生成新代码（新增模块、示例、测试、脚本、配置、模型文件、工具链脚本等）时，必须同步检查并更新 `.gitignore`：若新代码会产生构建产物、运行时日志、缓存、中间文件、生成数据或脚本输出，必须在提交前将对应路径或通配符加入 `.gitignore`，确保 `git status` 中不出现非预期的未跟踪文件。
6. 所有工作日志（如开发调试日志、性能记录、临时调试输出）、运行时输出（如图像保存、结果 CSV、调试截图、训练日志、推理日志），无论存放路径如何，一律不得纳入 Git 版本控制，必须通过 `.gitignore` 排除。
8. Pull Request 描述应包含需求编号、实现说明、测试命令、测试结果和效果截图/视频。
9. `main` 分支保持可构建；合并前必须完成对应自动测试与人工验收。
10. 不得提交对 `learning/` 的修改。

## 14. 实施里程碑

### M1：现有基础功能验收

- 验证 `myCamera` 在 Windows 和 Ubuntu 的真实取流。
- 运行现有自动测试。
- 使用实验室装甲板验证传统视觉框选、数字识别和跟踪。
- 建立参数配置和失败样例集。

### M2：YOLO 检测与跟踪

- 确认数据集、类别和许可证。
- 训练或选定模型，完成导出与部署。
- 实现项目自有 YOLO 封装并转换为统一检测结果。
- 接入跟踪器，完成精度和速度评估。

### M3：上下位机通信

- 确认 STM32、OLED、串口参数和 payload 字段。
- 完成面包板接线、PC 发送类和 STM32 帧解析。
- OLED 显示视觉结果并完成长时间稳定性测试。

### M4：工程化发挥项

- 按模块封装 `my-----` 类。
- 在回归测试保护下迁移为父子级 CMake。
- 建立 Windows/Ubuntu 自动构建。

### M5：旋转识别、距离和姿态

- 采集旋转装甲板数据并增强角点检测。
- 完成相机标定和装甲板尺寸确认。
- 实现 PnP、距离输出、重投影校验和三维坐标轴绘制。
- 使用实测距离和姿态进行量化验收。

## 15. 风险与待确认事项

### 15.1 已解决的风险（现已确认或冻结）

以下风险已在 2026-08-12 参数冻结中解决，详见 §3.1 参数冻结表：

| 原风险项 | 确认结果 |
|---|---|
| 数据集尚未确定 | YOLO 模型为公开/预训练模型，数据集来源和 License 待补充到文档 |
| 数字模型类别说明不完整 | 已确认 9 类: one~five, sentry, outpost, base, not_armor; 0-8 → `ArmorName` |
| 装甲板物理尺寸未知 | 已确认: 灯条 56mm, 小装甲板 135mm, 大装甲板 230mm |
| 串口数据字段未定义 | 已冻结: `docs/protocol.md` v1.0, 14 字节帧, 12B LE payload, XOR 校验 |
| STM32/OLED 型号未知 | 已确认: STM32F103C8T6, SSD1306 128×64 I2C (0x3C) |
| YOLO 仅输出矩形框 | 已确认 22 列输出格式含 8 个角点坐标 |
| FPS/识别率/距离误差等验收阈值未定义 | 已冻结全部验收阈值，见 §3.1 |

### 15.2 剩余风险

| 风险/待确认项 | 影响 | 处理方式 |
|---|---|---|
| 相机内参未标定 | 距离和姿态精度依赖 fallback 值 | 用实验室摄像头拍摄标定板，获取实际内参和畸变系数 |
| 光照与运动模糊 | 传统视觉漏检 | 参数化阈值、补光、曝光控制和数据增强 |
| 平面 PnP 多解与抖动 | 姿态跳变 | 重投影筛选、时序连续性和滤波 |
| Windows/Ubuntu 依赖差异 | 构建或运行不一致 | 目标级 CMake、CI 和真实设备双重测试 |

### 15.3 剩余 TBD

实施前必须完成的唯一确认项：

1. **相机标定**：使用实际摄像头重新标定，获取内参矩阵和畸变系数，替换 `calibration/camera_params.yaml` 中的 fallback 值（运行 `calibration/calibrate_camera.cpp` 完成标定）。

其余所有参数已冻结，详见 §3.1。后续如需调整，必须同步更新本文档的冻结参数表。

## 16. 最终交付物

- 可在 Windows 和 Ubuntu 构建的 C++17 源代码；
- `myCamera` 示例、传统视觉演示、YOLO 演示和串口联调程序；
- PC 端与 STM32 端一致的串口协议文档；
- STM32 工程、接线说明和 OLED 显示效果；
- 数据集来源、训练配置、模型及评估报告；
- 摄像头标定文件和距离/姿态误差报告；
- CMake 父子级构建文件与测试程序；
- GitHub 提交历史、构建说明和验收视频/截图；
- `learning/` 始终保持未修改状态的检查结果。
