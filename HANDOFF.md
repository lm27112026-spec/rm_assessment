# 题目2
# 装甲板视觉项目交接文档

## 1. 项目概述

本项目是一个基于 C++17 和 OpenCV/OpenVINO 的装甲板视觉演示程序，当前按功能模块拆分为 `MyCamera`、`MyArmorTraditional`、`MyArmorYolo`、`communication` 和 `stm32`，已实现：

1. 从 USB/内置摄像头、视频文件或网络视频源读取图像。
2. 根据灯条亮度和颜色信息提取候选灯条。
3. 对左右灯条进行几何配对，生成装甲板候选框和透视归一化 ROI。
4. 使用 Kalman Filter 风格的状态跟踪器，对连续帧中的目标进行跟踪和短时预测。
5. 使用 `learning/assets/tiny_resnet.onnx` 对装甲板数字/类别进行识别。
6. 在窗口中显示检测框、识别结果、跟踪状态、帧率等 HUD 信息。
7. 使用 YOLOv5 OpenVINO 模型完成题目3装甲板检测与跟随演示。
8. 使用 `communication/` 中的 `mySerial` 和帧协议完成上位机串口封帧发送。
9. 在无摄像头时使用 `demo.avi` 进行无界面冒烟测试。

重要约束：`learning/` 目录是只读输入，本次实现没有修改其中任何文件。

---

## 2. 目录和主要文件

```text
rm_assessment/
├─ CMakeLists.txt                 根 CMake，负责依赖查找和 add_subdirectory
├─ cmake/
│  └─ helpers.cmake               测试环境和运行时 DLL 辅助函数
├─ run.ps1                        Windows 构建和运行脚本
├─ HANDOFF.md                     本交接文档
├─ io/
│  └─ example.cpp                 题目1要求保留的 myCamera 示例
├─ MyCamera/
│  ├─ CMakeLists.txt
│  └─ myCamera.hpp/.cpp           跨平台图像采集封装
├─ MyArmorTraditional/
│  ├─ CMakeLists.txt
│  ├─ armor.hpp/.cpp              装甲板数据结构、候选和几何辅助
│  ├─ detector.hpp/.cpp           灯条检测、灯条配对和 ROI 生成
│  ├─ digit_recognizer.hpp/.cpp   ONNX 数字识别器
│  └─ tracker.hpp/.cpp            连续帧跟踪器
├─ MyArmorYolo/
│  ├─ CMakeLists.txt
│  ├─ yolov5.hpp/.cpp             YOLOv5 OpenVINO 推理封装
│  └─ yolov5_utils.hpp/.cpp       letterbox、decode、NMS 等工具
├─ communication/
│  ├─ CMakeLists.txt
│  ├─ frame.hpp/.cpp              0xAA + payload + 0xBB 帧协议
│  └─ mySerial.hpp/.cpp           PC 端串口封装
├─ stm32/                         下位机 UART/OLED 工程与说明
├─ tasks/
│  ├─ CMakeLists.txt
│  ├─ armor_demo.cpp              题目2传统视觉演示
│  ├─ yolov5_armor_demo.cpp       题目3 YOLO 演示
│  ├─ serial_demo.cpp             视觉到串口联调演示
│  └─ serial_loopback.cpp         串口回环测试程序
├─ tests/
│  ├─ CMakeLists.txt
│  ├─ armor_vision_test.cpp       检测器、识别器和跟踪器测试
│  ├─ myCamera_interface_test.cpp myCamera 编译期接口测试
│  ├─ myCamera_test.cpp           摄像头基础测试
│  └─ yolov5_test.cpp             YOLO 后处理和模型输出契约测试
├─ models/yolov5/                 YOLOv5 OpenVINO 模型
└─ learning/                      已有模型和示例素材，不要修改
   └─ assets/
      ├─ tiny_resnet.onnx         9 类装甲板分类模型
      └─ demo/demo.avi            示例视频
```

---

## 3. 已实现功能

### 3.1 图像输入

`armor_demo` 支持以下输入形式：

- 摄像头索引，例如 `0`、`1`。
- 视频文件，例如 `learning/assets/demo/demo.avi`。
- OpenCV 能打开的 URL/网络视频源。

默认输入源为摄像头 `0`。Windows 下程序使用 Media Foundation（`cv::CAP_MSMF`）尝试打开摄像头，并自动扫描索引 `0–5`。

如果没有摄像头，会输出：

```text
No available camera was found (checked indices 0-5).
```

### 3.2 灯条和装甲板检测

检测流程大致如下：

```text
输入帧
  ↓
颜色/亮度分割
  ↓
形态学处理
  ↓
轮廓提取
  ↓
灯条尺寸、长宽比、角度过滤
  ↓
左右灯条几何配对
  ↓
装甲板候选 ArmorCandidate
  ↓
透视变换，生成 normalized_roi
```

候选中包含：

- 装甲板颜色：红、蓝或未知。
- 四个角点。
- 中心点。
- 外接矩形。
- 归一化透视 ROI。

检测器通过以下几何因素排除误检：

- 灯条轮廓面积。
- 灯条长度和长宽比。
- 左右灯条长度比例。
- 两灯条之间的距离比例。
- 灯条相对角度。
- 灯条中心高度差。
- 装甲板整体长宽比。

### 3.3 ONNX 数字识别

原先的模板匹配方案依赖 `1.png` 到 `5.png`，但仓库中没有实际模板图片，因此已经替换为 OpenCV DNN ONNX 推理。

模型路径：

```text
learning/assets/tiny_resnet.onnx
```

模型类别顺序由原项目代码确认：

```text
0: one
1: two
2: three
3: four
4: five
5: sentry
6: outpost
7: base
8: not_armor
```

当前 `DigitRecognizer` 对前五类返回数字 `1–5`，对 `sentry`、`outpost`、`base` 和 `not_armor` 返回 `Unknown`。

模型预处理严格遵循原分类器：

1. 输入 ROI 转为灰度图。
2. 按比例缩放，使 ROI 完整放入 `32×32`。
3. 将缩放结果放到黑色 `32×32` 画布左上角。
4. 使用 `blobFromImage(input, 1.0 / 255.0, ...)` 归一化。
5. 调用 `net.forward()` 得到 9 个 logits。
6. 使用数值稳定的 softmax 计算概率。

如果模型文件不存在、模型加载失败、输入为空、输出尺寸不正确或推理抛出异常，识别器会安全返回：

```text
label      = Unknown
digit      = -1
confidence = 0
reliable   = false
```

### 3.4 连续跟踪

`Tracker` 根据检测到的装甲板候选维护目标状态，支持以下状态：

- `lost`：当前没有目标。
- `detecting`：刚检测到目标，正在确认。
- `tracking`：目标稳定跟踪中。
- `temp_lost`：短时间丢失，使用预测结果维持目标。

跟踪器可以在短暂漏检时继续输出预测框，超过丢失阈值后回到 `lost`。

### 3.5 可视化和统计

窗口模式下会显示：

- 装甲板四边形和中心点。
- 识别标签和置信度。
- 跟踪预测框。
- 来源、模型加载状态、亮度阈值、FPS、帧号和跟踪状态。

无界面模式会输出：

```text
frames=...
candidates=...
tracker_frames=...
reliable_recognitions=...
```

---

## 4. 环境要求

- Windows。
- Visual Studio/MSVC，支持 C++17。
- CMake 3.16 或更高版本。
- OpenCV 4.10.0，当前配置路径为：

```text
D:\OpenCV\opencv\build
```

- OpenCV DLL 目录：

```text
D:\OpenCV\opencv\build\x64\vc16\bin
```

其中应包含：

```text
opencv_world4100.dll
```

---

## 5. 构建代码

### 5.1 使用 CMake 手动构建

在项目根目录执行：

```powershell
cmake -S . -B build -DOpenCV_DIR="D:\OpenCV\opencv\build"
cmake --build build --config Release
```

生成的主要程序：

```text
build\tasks\Release\armor_demo.exe
build\tests\Release\armor_vision_test.exe
build\tests\Release\mycamera_test.exe
```

启用 YOLO/OpenVINO 时使用：

```powershell
cmake -S . -B build -DOpenCV_DIR="D:\OpenCV\opencv\build" `
  -DYOLO_WITH_OPENVINO=ON `
  -DOpenVINO_DIR="D:\python\Lib\site-packages\openvino\cmake"
cmake --build build --config Release
```

### 5.2 使用 run.ps1 构建并运行

`run.ps1` 会自动构建目标，并把 OpenCV DLL 目录加入当前进程的 `PATH`。

通用格式：

```powershell
.\run.ps1 <Target> <Config> [程序参数...]
```

例如：

```powershell
.\run.ps1 armor_demo Release
```

如果 PowerShell 禁止执行脚本：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

---

## 6. 运行程序

### 6.1 摄像头窗口模式

推荐先使用较低亮度阈值：

```powershell
.\run.ps1 armor_demo Release 0 80
```

参数含义：

```text
0   摄像头索引
80  brightness_threshold
```

程序默认会加载：

```text
learning/assets/tiny_resnet.onnx
```

退出方式：

- 按 `q`。
- 按 `Esc`。

### 6.2 摄像头无界面模式

```powershell
.\run.ps1 armor_demo Release --headless --max-frames 300 0 80
```

无界面模式适合检查检测数量和跟踪数量，不会显示框选窗口。

### 6.3 使用示例视频

没有摄像头时可以运行：

```powershell
.\run.ps1 armor_demo Release --headless --max-frames 300 `
  learning\assets\demo\demo.avi
```

也可以直接运行已构建程序，但必须先设置 DLL 路径：

```powershell
$env:Path = "D:\OpenCV\opencv\build\x64\vc16\bin;$env:Path"
& .\build\tasks\Release\armor_demo.exe --headless --max-frames 300 `
  learning\assets\demo\demo.avi
```

视频结束后程序会退出。若看到 MJPEG 尾部 `overread` 警告，通常是示例视频尾部编码警告，不代表前面的帧无法解码。

### 6.4 自定义模型路径

参数格式为：

```powershell
.\run.ps1 armor_demo Release <source> <model_path> <brightness_threshold>
```

例如：

```powershell
.\run.ps1 armor_demo Release 0 learning\assets\tiny_resnet.onnx 80
```

模型路径不存在时，程序仍可运行检测和跟踪，但识别结果全部为 `Unknown`。

### 6.5 运行 mycamera_test

```powershell
.\run.ps1 mycamera_test Release
```

不要直接双击或直接运行：

```powershell
.\build\tests\Release\mycamera_test.exe
```

否则 Windows 可能报：

```text
找不到 opencv_world4100.dll
```

如果必须直接运行，先设置：

```powershell
$env:Path = "D:\OpenCV\opencv\build\x64\vc16\bin;$env:Path"
& .\build\tests\Release\mycamera_test.exe
```

另外，`mycamera_test` 可能会等待真实摄像头输入。没有摄像头时看起来像长时间不退出，这是输入设备问题，不是 DLL 问题。

---

## 7. 测试和验证

### 7.1 运行装甲视觉测试

推荐使用 CTest：

```powershell
ctest --test-dir build -C Release -R armor_vision_test --output-on-failure
```

当前测试覆盖：

- 合成蓝色灯条可以被检测。
- 缺失 ONNX 模型时识别器安全返回 `Unknown`。
- `tiny_resnet.onnx` 可以加载。
- 识别置信度处于 `[0, 1]`。
- 返回数字时范围为 `1–5`。
- Tracker 的 `lost → detecting → tracking → temp_lost → lost` 状态转换。

### 7.2 运行全部 CTest

```powershell
$env:Path = "D:\OpenCV\opencv\build\x64\vc16\bin;$env:Path"
ctest --test-dir build -C Release --output-on-failure
```

如果 `mycamera_test` 等待摄像头导致超时，应单独运行：

```powershell
ctest --test-dir build -C Release -R armor_vision_test --output-on-failure
```

### 7.3 视频冒烟测试

```powershell
$env:Path = "D:\OpenCV\opencv\build\x64\vc16\bin;$env:Path"
& .\build\tasks\Release\armor_demo.exe --headless --max-frames 30 `
  learning\assets\demo\demo.avi
```

此前验证结果示例：

```text
Opened source: learning\assets\demo\demo.avi
Digit model: ...\learning\assets\tiny_resnet.onnx (loaded)
frames=30 candidates=29 tracker_frames=28 reliable_recognitions=0
```

### 7.4 确认 learning 未被修改

```powershell
git diff --quiet -- learning
```

返回成功表示 `learning/` 没有未提交修改。

---

## 8. 摄像头和阈值调节注意事项

### 8.1 亮度阈值是最重要的现场参数

当前程序的亮度阈值默认值为 `150`，但不同摄像头、曝光、距离和环境光下不一定合适。

如果完全检测不到装甲板，依次尝试：

```powershell
.\run.ps1 armor_demo Release 0 60
.\run.ps1 armor_demo Release 0 80
.\run.ps1 armor_demo Release 0 100
.\run.ps1 armor_demo Release 0 120
```

选择能稳定检测且误检较少的值。

本次现场现象表明，降低阈值后可以恢复检测，因此主要原因是当前摄像头画面亮度与默认阈值不匹配。

### 8.2 什么时候需要调整摄像头参数

通常不必先修改摄像头底层参数，优先调整程序亮度阈值。

只有出现以下情况时，才建议调整曝光、增益、白平衡或分辨率：

- 远距离灯条完全不亮：提高曝光或增益。
- 灯条过曝并扩散：降低曝光或增益。
- 画面颜色随时间明显变化：关闭自动白平衡并固定白平衡。
- 画面亮暗周期性变化：关闭自动曝光并固定曝光。
- 运动拖影严重：缩短曝光时间，必要时提高增益。

### 8.3 颜色和光源要求

检测器使用 BGR 图像和红/蓝颜色差异进行分割。现场应注意：

- 摄像头输出必须是正常 BGR 图像，不能误传 RGB 顺序。
- 灯条颜色应与检测器的红/蓝判定一致。
- 强环境光、反光、过曝可能产生误检。
- 黑色背景通常比复杂背景更容易检测。

---

## 9. 已知限制

1. 当前没有真实 USB 摄像头可供持续自动化验证，摄像头效果需要现场调参。
2. Windows 摄像头是否可用受隐私权限、设备管理器、硬件开关和驱动影响。
3. `tiny_resnet.onnx` 的普通数字类别是 `1–5`；特殊类别不会转成数字，而是返回 `Unknown`。
4. 识别器只负责对检测器生成的 `normalized_roi` 分类；如果装甲板检测失败，识别器不会单独从整帧搜索目标。
5. 当前 demo 的稳定性依赖灯条尺寸、亮度和几何形状。不同相机分辨率或镜头视场可能需要调整检测器参数。
6. `mycamera_test` 依赖真实摄像头输入，不能作为无硬件环境下的快速单元测试。
7. 直接运行 exe 需要 OpenCV DLL 在 `PATH` 中；使用 `run.ps1` 可以避免这个问题。
8. 构建时可能出现已有源文件中的 MSVC `C4819` 中文编码警告。该警告不影响当前编译和运行，但后续可统一源文件编码。

---

## 10. 后续建议

如果继续完善项目，建议按以下顺序处理：

1. 增加摄像头画面实时参数调节 UI 或命令行循环调参。
2. 将检测候选数量、亮度分割图和灯条轮廓提供调试窗口。
3. 为不同分辨率和不同摄像头保存现场样本，补充真实数据测试。
4. 将 `mycamera_test` 改造成可配置超时的摄像头测试，避免无硬件时阻塞整个 CTest。
5. 将 OpenCV DLL 的运行时部署改为复制到构建输出目录，减少对环境变量的依赖。
6. 如果需要识别 `sentry`、`outpost`、`base`，应扩展 `RecognitionResult` 的类别语义，而不是把这些类别强行转换为数字。

---

## 11. 快速启动清单

在有摄像头的 Windows 环境中：

```powershell
cd D:\Desktop\rm_assessment
Set-ExecutionPolicy -Scope Process Bypass
.\run.ps1 armor_demo Release 0 80
```

没有摄像头时：

```powershell
cd D:\Desktop\rm_assessment
Set-ExecutionPolicy -Scope Process Bypass
.\run.ps1 armor_demo Release --headless --max-frames 300 `
  learning\assets\demo\demo.avi
```

验证测试：

```powershell
ctest --test-dir build -C Release -R armor_vision_test --output-on-failure
```
