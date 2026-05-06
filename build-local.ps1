#Requires -Version 5.1
<#
.SYNOPSIS
    Local build script for the full FSMP mod (DLL + MCM + package).
.DESCRIPTION
    Mirrors the GitHub Actions build.yml pipeline for local development.
    Builds one CMake preset and compiles the Papyrus MCM scripts, then
    assembles the mod package.

    Prerequisites:
    - CMakeUserPresets.json defining a local preset (e.g. vs2026-windows)
    - git submodules initialized (run: git submodule update --init --recursive)
    - FSMP-MCM submodules initialized (same command in FSMP-MCM dir)

.PARAMETER Preset
    CMake preset to build. Defaults to "vs2026-windows".
    Must be defined in CMakeUserPresets.json (or CMakePresets.json).
.PARAMETER McmDir
    Path to the FSMP-MCM repository. Defaults to sibling folder FSMP-MCM.
.PARAMETER ValidatorDir
    Path to the FSMP-Validator repository. Defaults to sibling folder FSMP-Validator.
.PARAMETER OutputDir
    Where to place the final package. Defaults to "out/package" under this repo.
.EXAMPLE
    .\build-local.ps1
.EXAMPLE
    .\build-local.ps1 -Preset vs2022-windows
#>
param(
    [string]$Preset      = "vs2026-windows",
    [string]$McmDir      = (Join-Path (Split-Path $PSScriptRoot -Parent) "FSMP-MCM"),
    [string]$ValidatorDir= (Join-Path (Split-Path $PSScriptRoot -Parent) "FSMP-Validator"),
    [string]$OutputDir   = (Join-Path $PSScriptRoot "out\package")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepoDir = $PSScriptRoot
$BuildDir = Join-Path $RepoDir "out\build\$Preset"

function Write-Step { param([string]$msg) Write-Host "`n=== $msg ===" -ForegroundColor Cyan }
function Write-OK   { param([string]$msg) Write-Host "  OK: $msg" -ForegroundColor Green }
function Invoke-Cmd {
    param([string]$Exe, [string[]]$Args, [string]$WorkDir = $RepoDir)
    Write-Host "  > $Exe $($Args -join ' ')" -ForegroundColor DarkGray
    Push-Location $WorkDir
    try { & $Exe @Args; if ($LASTEXITCODE -ne 0) { throw "Command failed with exit code $LASTEXITCODE" } }
    finally { Pop-Location }
}

# ---------------------------------------------------------------------------
# 1. Ensure plugins output dir exists (cmake post-build copies DLL there)
# ---------------------------------------------------------------------------
$PluginsDir = Join-Path $BuildDir "plugins\SKSE\Plugins"
New-Item -ItemType Directory -Force -Path $PluginsDir | Out-Null

# ---------------------------------------------------------------------------
# 2. CMake configure
# ---------------------------------------------------------------------------
Write-Step "CMake configure ($Preset)"
Invoke-Cmd cmake @("--preset", $Preset)

# ---------------------------------------------------------------------------
# 3. CMake build (Release)
# ---------------------------------------------------------------------------
Write-Step "CMake build ($Preset / Release)"
Invoke-Cmd cmake @("--build", "out\build\$Preset", "--config", "Release")

$DllPath = Join-Path $PluginsDir "hdtsmp64.dll"
if (-not (Test-Path $DllPath)) { throw "DLL not found at $DllPath" }
Write-OK "DLL built: $DllPath"

# ---------------------------------------------------------------------------
# 4. Compile MCM Papyrus scripts
# ---------------------------------------------------------------------------
Write-Step "Compile MCM Papyrus scripts"
if (-not (Test-Path $McmDir)) { throw "FSMP-MCM not found at '$McmDir'" }

$CompileScript = Join-Path $McmDir "dev-scripts\compile.ps1"
if (-not (Test-Path $CompileScript)) { throw "compile.ps1 not found at '$CompileScript'" }

Write-Host "  > $CompileScript -Mode Release" -ForegroundColor DarkGray
& $CompileScript -Mode Release
if ($LASTEXITCODE -ne 0) { throw "MCM compile failed with exit code $LASTEXITCODE" }

$PexFiles = Get-ChildItem (Join-Path $McmDir "Scripts\*.pex") -ErrorAction SilentlyContinue
Write-OK "MCM compiled: $($PexFiles.Count) .pex file(s)"

# ---------------------------------------------------------------------------
# 5. Assemble package
# ---------------------------------------------------------------------------
Write-Step "Assembling package -> $OutputDir"

if (Test-Path $OutputDir) { Remove-Item -Recurse -Force $OutputDir }
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

# 5a. fomod tree content → root of package
Copy-Item -Path (Join-Path $RepoDir "fomod tree\*") -Destination $OutputDir -Recurse -Force

# 5b. DLL artifacts → raw-$Preset sub-folder (matching CI naming)
$RawDir = Join-Path $OutputDir "raw-$Preset"
Copy-Item -Path (Join-Path $BuildDir "plugins") -Destination $RawDir -Recurse -Force

# 5c. Validator files → FSMPV/
$FsmpvDir = Join-Path $OutputDir "FSMPV"
New-Item -ItemType Directory -Force -Path $FsmpvDir | Out-Null
if (Test-Path $ValidatorDir) {
    $licPath = Join-Path $ValidatorDir "LICENSE"
    if (Test-Path $licPath) { Copy-Item $licPath $FsmpvDir }
    $xsdPath = Join-Path $ValidatorDir "skse\plugins\hdtSkinnedMeshConfigs\hdtSMP64.xsd"
    if (Test-Path $xsdPath) { Copy-Item $xsdPath $FsmpvDir }
    $schPath = Join-Path $ValidatorDir "skse\plugins\hdtSkinnedMeshConfigs\hdtSMP64.sch"
    if (Test-Path $schPath) { Copy-Item $schPath $FsmpvDir }
    Write-OK "Validator files copied"
} else {
    Write-Warning "FSMP-Validator not found at '$ValidatorDir', skipping validator files"
}

# 5d. MCM files → FSMPM/
$FsmpmDir = Join-Path $OutputDir "FSMPM"
New-Item -ItemType Directory -Force -Path $FsmpmDir | Out-Null
Get-ChildItem (Join-Path $McmDir "FSMPM*") -ErrorAction SilentlyContinue | Copy-Item -Destination $FsmpmDir
$mcmLic = Join-Path $McmDir "LICENSE"
if (Test-Path $mcmLic) { Copy-Item $mcmLic $FsmpmDir }
foreach ($sub in @("SKSE", "Scripts", "Source")) {
    $src = Join-Path $McmDir $sub
    if (Test-Path $src) { Copy-Item $src $FsmpmDir -Recurse -Force }
}
$ifaceDir = Join-Path $McmDir "interface"
if (Test-Path $ifaceDir) { Copy-Item $ifaceDir $FsmpmDir -Recurse -Force }
Write-OK "MCM files copied"

# ---------------------------------------------------------------------------
# 6. Create ZIP (if 7z is available)
# ---------------------------------------------------------------------------
Write-Step "Creating ZIP archive"
$_7zCmd = Get-Command "7z.exe" -ErrorAction SilentlyContinue
$SevenZip = if ($_7zCmd) { $_7zCmd.Source } else { $null }
if (-not $SevenZip) {
    # Common install locations
    foreach ($p in @("C:\Program Files\7-Zip\7z.exe","C:\Program Files (x86)\7-Zip\7z.exe")) {
        if (Test-Path $p) { $SevenZip = $p; break }
    }
}

# Read version from CMakeLists.txt
$BaseVersion = (Select-String -Path (Join-Path $RepoDir "CMakeLists.txt") -Pattern 'VERSION\s+(\d+\.\d+\.\d+)' | Select-Object -First 1).Matches.Groups[1].Value
$ShortSha = (git -C $RepoDir rev-parse --short HEAD 2>$null) -replace "`n",""
$BranchRaw = (git -C $RepoDir rev-parse --abbrev-ref HEAD 2>$null) -replace "`n",""
$Branch = $BranchRaw -replace "/", "-"
$TagVersion = if ($Branch -eq "master") { $BaseVersion } else { "$BaseVersion-$Branch-$ShortSha" }
$ZipName = "FSMP-$TagVersion.zip"
$ZipPath = Join-Path (Split-Path $OutputDir -Parent) $ZipName

if ($SevenZip) {
    if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
    Invoke-Cmd $SevenZip @("a", "-tzip", "-mm=LZMA", "-mx=9", $ZipPath, ".") -WorkDir $OutputDir
    Write-OK "Package: $ZipPath"
} else {
    Write-Warning "7z.exe not found - creating .zip with Compress-Archive instead"
    if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
    Compress-Archive -Path (Join-Path $OutputDir "*") -DestinationPath $ZipPath
    Write-OK "Package: $ZipPath"
}

Write-Host "`n=== BUILD COMPLETE ===" -ForegroundColor Green
Write-Host "  Package: $ZipPath"
Write-Host "  DLL:     $DllPath"
