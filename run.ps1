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
$ExePath = Join-Path $BuildDir "$Config\$Target.exe"

if (-not (Test-Path -LiteralPath $BuildDir)) {
    cmake -S $Root -B $BuildDir -DOpenCV_DIR=$OpenCvDir
}

cmake --build $BuildDir --config $Config --target $Target

if (-not (Test-Path -LiteralPath $ExePath)) {
    throw "Executable not found: $ExePath"
}

$env:Path = "$OpenCvBin;$env:Path"
& $ExePath @ProgramArgs
