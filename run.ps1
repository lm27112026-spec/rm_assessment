param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Target,

    [Parameter(Position = 1)]
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Debug",

    [Parameter(Position = 2, ValueFromRemainingArguments = $true)]
    [string[]]$ProgramArgs
)

$ErrorActionPreference = "Stop"

$Root = $PSScriptRoot
$BuildDir = Join-Path $Root "build-opencv"
$OpenCvDir = "D:\OpenCV\opencv\build"
$OpenCvBin = "D:\OpenCV\opencv\build\x64\vc16\bin"
$OpenVinoDir = "D:\python\Lib\site-packages\openvino\cmake"
$OpenVinoBin = "D:\python\Lib\site-packages\openvino\libs"
$NeedsYolo = $Target -eq "yolov5_armor_demo" -or $Target -eq "yolov5_pnp_demo" -or $Target -eq "yolov5_test"
$ExeCandidates = @(
    (Join-Path $BuildDir "$Config\$Target.exe"),
    (Join-Path $BuildDir "MyCamera\$Config\$Target.exe"),
    (Join-Path $BuildDir "tasks\$Config\$Target.exe"),
    (Join-Path $BuildDir "tests\$Config\$Target.exe"),
    (Join-Path $BuildDir "communication\$Config\$Target.exe")
)

if ($NeedsYolo) {
    cmake -S $Root -B $BuildDir `
        "-DOpenCV_DIR=$OpenCvDir" `
        -DYOLO_WITH_OPENVINO=ON `
        "-DOpenVINO_DIR=$OpenVinoDir"
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configure failed for '$Target'."
    }
} elseif (-not (Test-Path -LiteralPath $BuildDir)) {
    cmake -S $Root -B $BuildDir "-DOpenCV_DIR=$OpenCvDir"
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configure failed."
    }
}

cmake --build $BuildDir --config $Config --target $Target
if ($LASTEXITCODE -ne 0) {
    throw "Build failed for target '$Target'."
}

$ExePath = $ExeCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $ExePath) {
    throw "Executable not found for target '$Target'. Checked: $($ExeCandidates -join ', ')"
}

$env:Path = "$OpenCvBin;$OpenVinoBin;$env:Path"
& $ExePath @ProgramArgs
