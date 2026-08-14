# 题目2
# 装甲板视觉项目交接文档

## 1. 项目概述

本项目是一个基于 C++17 和 OpenCV/OpenVINO 的装甲板视觉演示程序，当前按功能模块拆分为 `MyCamera`、`MyArmorTraditional`、`MyArmorYolo`、`communication` 和 `stm32`，已实现：

1. 从 USB/内置摄像头、视频文件或网络视频源读取图像。
2. 根据灯条亮度和颜色信息提取候选灯条。
3. 对左右灯条进行几何配对，生成装甲板候选（颜色、四角点、中心）。
4. 使用 `learning/assets/tiny_resnet.onnx` 对装甲板做 9 类分类，并作为检测门控过滤误检。
5. YOLO demo 使用轻量卡尔曼跟踪器对目标进行短时预测。
6. 在窗口中显示检测框、识别结果、帧率等 HUD 信息。
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
│  ├─ armor.hpp                   装甲板/灯条数据结构（auto_aim 命名空间）
│  ├─ detector.hpp/.cpp           检测 + ONNX 分类一体（auto_aim::Detector）
│  └─ img_tools.hpp               绘图工具（tools 命名空间）
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
│  ├─ yolov5_armor_demo.cpp       题目3 YOLO 检测 + 跟随演示
│  ├─ detection_tracker.hpp       YOLO 演示用轻量卡尔曼跟踪器
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

检测流程（`auto_aim::Detector`）大致如下：

```text
输入帧
  ↓
灰度化 + 固定阈值(120)二值化
  ↓
findContours 轮廓提取
  ↓
minAreaRect 拟合灯条，几何过滤（角度误差<45°、长宽比 1.5~20、长度>8）
  ↓
统计轮廓红/蓝通道判定灯条颜色
  ↓
同色灯条从左到右两两配对，几何校验（间距比 1~5、侧边比<1.5、矩形度误差<25°）
  ↓
沿灯条方向扩展得到装甲板 ROI，送入 ONNX 分类
  ↓
置信度>0.8 且类别不是 not_armor 才保留为装甲板 Armor
```

每个装甲板 `auto_aim::Armor` 包含：

- 颜色（红/蓝）。
- 四角点 `points` 与中心 `center`。
- 分类类别 `name`（one~five / sentry / outpost / base / not_armor）与置信度 `confidence`。

与上一版相比，关键区别是把 `tiny_resnet.onnx` 分类作为检测门控（`confidence > 0.8 && name != not_armor`），从源头过滤掉非装甲板的误检。

### 3.3 ONNX 数字/类别识别

识别已整合进 `auto_aim::Detector`，不再是独立的 `DigitRecognizer` 类。模型：

```text
learning/assets/tiny_resnet.onnx
```

9 个类别顺序：

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

预处理严格遵循原分类器：

1. 装甲板 ROI 转为灰度图。
2. 按比例缩放，使 ROI 完整放入 `32×32`。
3. 将缩放结果放到黑色 `32×32` 画布左上角。
4. 使用 `blobFromImage(input, 1.0 / 255.0, ...)` 归一化。
5. 调用 `net.forward()` 得到 9 个 logits。
6. 使用数值稳定的 softmax 计算概率，取最大类别。

分类结果写入 `Armor::name`（`ArmorName` 枚举）与 `Armor::confidence`。`Detector` 在构造时加载模型一次（路径可配，默认 `learning/assets/tiny_resnet.onnx`）。若模型缺失，分类返回 `not_armor`，由于识别作为检测门控，检测结果为空。

### 3.4 连续跟踪

传统视觉 demo（`armor_demo`）不再使用卡尔曼跟踪，直接输出每帧检测结果。

YOLO demo（`yolov5_armor_demo`）使用 `tasks/detection_tracker.hpp` 中的轻量跟踪器
`rm_assessment::DetectionTracker`（四维恒速卡尔曼，跟踪框中心），状态：

- `lost`：当前没有目标。
- `tracking`：目标稳定跟踪中。
- `temp_lost`：短时间丢失，使用预测框维持输出，超过阈值后回到 `lost`。

跟踪器在短暂漏检时继续输出预测框，超过丢失阈值后回到 `lost`。

### 3.5 可视化和统计

窗口模式下会显示：

- 装甲板四边形和中心点。
- 类别标签（颜色、ArmorName）和置信度。
- 来源、模型加载状态、FPS、帧号和检测数量。

无界面模式会输出：

```text
frames=...
detections=...
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

默认打开摄像头 `0`，并加载 `learning/assets/tiny_resnet.onnx`：

```powershell
.\run.ps1 armor_demo Release 0
```

参数含义：

```text
0   摄像头索引（可选，默认 0）
```

退出方式：

- 按 `q`。
- 按 `Esc`。

### 6.2 使用示例视频

没有摄像头时可以运行：

```powershell
.\run.ps1 armor_demo Release --max-frames 300 learning\assets\demo\demo.avi
```

也可以直接运行已构建程序（OpenCV/MinGW DLL 已自动复制到 exe 同目录，无需手动设置 PATH）：

```powershell
& .\build\tasks\armor_demo.exe --max-frames 300 learning\assets\demo\demo.avi
```

视频结束后程序会退出。若看到 MJPEG 尾部 `overread` 警告，通常是示例视频尾部编码警告，不代表前面的帧无法解码。

### 6.3 自定义模型路径

参数格式为：

```powershell
.\run.ps1 armor_demo Release <source> [model_path]
```

例如：

```powershell
.\run.ps1 armor_demo Release 0 learning\assets\tiny_resnet.onnx
```

模型路径不存在时，`Detector::has_model()` 返回 false；由于分类作为检测门控，此时检测结果为空。

### 6.4 运行 mycamera_test

```powershell
.\run.ps1 mycamera_test Release
```

构建时已自动把 OpenCV 与运行时 DLL 复制到 exe 同目录（`copy_opencv_runtime_dlls`），可直接运行：

```powershell
& .\build\tests\mycamera_test.exe
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

- 缺失 ONNX 模型时检测器安全返回空结果。
- `tiny_resnet.onnx` 可以加载。
- 有模型时空图不产生误检。

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
& .\build\tasks\armor_demo.exe --max-frames 30 learning\assets\demo\demo.avi
```

验证结果示例：

```text
Opened source: learning\assets\demo\demo.avi
Digit model: ...\learning\assets\tiny_resnet.onnx (loaded)
frames=30 detections=0
```

### 7.4 确认 learning 未被修改

```powershell
git diff --quiet -- learning
```

返回成功表示 `learning/` 没有未提交修改。

---

## 8. 摄像头和阈值调节注意事项

### 8.1 二值化阈值

当前检测器在 `src/MyArmorTraditional/detector.cpp` 中固定使用灰度阈值 `120` 二值化（`cv::threshold(..., 120, 255, THRESH_BINARY)`）。不同摄像头、曝光、距离和环境光下可能需要调整此常量。

如果完全检测不到装甲板，可修改该阈值后重新构建，或优先调整摄像头参数。

### 8.2 什么时候需要调整摄像头参数

出现以下情况时，建议调整曝光、增益、白平衡或分辨率：

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
3. `tiny_resnet.onnx` 输出 9 类（one~five / sentry / outpost / base / not_armor），结果写入 `Armor::name`。
4. 分类作为检测门控整合在 `Detector` 内；若模型缺失，检测将不输出任何结果（而非输出无标签候选）。
5. 当前 demo 的稳定性依赖灯条尺寸、亮度和几何形状（二值化阈值等常量见 `detector.cpp`）。不同相机分辨率或镜头视场可能需要调参。
6. `mycamera_test` 依赖真实摄像头输入，不能作为无硬件环境下的快速单元测试。
7. 构建时已自动复制 OpenCV/MinGW 运行时 DLL 到 exe 同目录（`copy_opencv_runtime_dlls`），无需手动设置 PATH。
8. 构建时可能出现已有源文件中的 MSVC `C4819` 中文编码警告。该警告不影响当前编译和运行，但后续可统一源文件编码。

---

## 10. 后续建议

如果继续完善项目，建议按以下顺序处理：

1. 增加摄像头画面实时参数调节 UI 或命令行循环调参。
2. 将检测候选数量、亮度分割图和灯条轮廓提供调试窗口。
3. 为不同分辨率和不同摄像头保存现场样本，补充真实数据测试。
4. 将 `mycamera_test` 改造成可配置超时的摄像头测试，避免无硬件时阻塞整个 CTest。
5. （已完成）OpenCV DLL 已通过 `copy_opencv_runtime_dlls` 自动复制到输出目录。
6. 当前 `Armor::name` 已保留 9 类语义（含 `sentry`/`outpost`/`base`）；若串口协议需要区分这些类别，应扩展 `frame::Payload` 的字段，而不是把这些类别强行映射为数字。

---

## 11. 快速启动清单

在有摄像头的 Windows 环境中：

```powershell
cd D:\Desktop\rm_assessment
Set-ExecutionPolicy -Scope Process Bypass
.\run.ps1 armor_demo Release 0
```

没有摄像头时：

```powershell
cd D:\Desktop\rm_assessment
Set-ExecutionPolicy -Scope Process Bypass
.\run.ps1 armor_demo Release --max-frames 300 learning\assets\demo\demo.avi
```

验证测试：

```powershell
ctest --test-dir build -C Release -R armor_vision_test --output-on-failure
```
