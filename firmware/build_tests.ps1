# Build and run the host-side firmware tests with gcc, then the golden check.
# Usage (any directory):  powershell -ExecutionPolicy Bypass -File firmware\build_tests.ps1
# Optional: -Gcc <path to gcc.exe> -Python <path to python.exe>
param(
    [string]$Gcc = "gcc",
    [string]$Python = "python"
)
$ErrorActionPreference = "Stop"

$fw = $PSScriptRoot
$repo = Split-Path $fw -Parent
$build = Join-Path $fw "build"
New-Item -ItemType Directory -Force $build | Out-Null

$cflags = @("-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2",
            "-I$fw\protocol", "-I$fw\drivers", "-I$fw\tests")

function Invoke-Gcc([string]$out, [string[]]$srcs) {
    & $Gcc @cflags -o (Join-Path $build $out) @srcs
    if ($LASTEXITCODE -ne 0) { throw "gcc failed for $out" }
}

Invoke-Gcc "test_frame.exe"   @("$fw\protocol\frame.c", "$fw\tests\test_frame.c")
Invoke-Gcc "test_mpu6050.exe" @("$fw\drivers\mpu6050.c", "$fw\tests\test_mpu6050.c")
Invoke-Gcc "gen_frames.exe"   @("$fw\protocol\frame.c", "$fw\drivers\mpu6050.c", "$fw\tests\gen_frames.c")

$failed = 0
foreach ($t in @("test_frame.exe", "test_mpu6050.exe")) {
    & (Join-Path $build $t)
    if ($LASTEXITCODE -ne 0) { $failed++ }
}

$golden = Join-Path $build "golden_frames.bin"
& (Join-Path $build "gen_frames.exe") $golden
if ($LASTEXITCODE -ne 0) { $failed++ }

Push-Location $repo
try {
    $env:PYTHONPATH = $repo
    & $Python (Join-Path $fw "tests\check_golden.py") $golden
    if ($LASTEXITCODE -ne 0) { $failed++ }
} finally {
    Pop-Location
}

if ($failed -ne 0) { Write-Host "FAILED: $failed step(s)"; exit 1 }
Write-Host "All firmware host tests passed."
