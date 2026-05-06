#Requires -Version 5.1
<#
.SYNOPSIS
    Local build script for the full FSMP mod (DLL x4 + MCM + package).
.DESCRIPTION
    Mirrors the GitHub Actions build.yml pipeline for local development.
    Builds all four AVX variants (noavx, avx, avx2, avx512) using local
    CMake presets, compiles the Papyrus MCM scripts, and assembles the
    final mod package.

    The DLL artifacts are placed in "raw-vs2022-*" directories inside the
    package, matching the names used by ModuleConfig.xml and the CI pipeline,
    even though locally they are built with VS 2026.

    Prerequisites:
    - CMakeUserPresets.json defining local vs2026-windows* presets (gitignored)
    - git submodules initialized: git submodule update --init --recursive
    - FSMP-MCM submodules initialized (same command in the FSMP-MCM dir)

.PARAMETER Variants
    List of AVX variant suffixes to build. Defaults to all four.
    Each entry maps to a "vs2026-windows{suffix}" local preset and a
    "raw-vs2022-windows{suffix}" output directory in the package.
.PARAMETER McmDir
    Path to the FSMP-MCM repository. Defaults to sibling folder FSMP-MCM.
.PARAMETER ValidatorDir
    Path to the FSMP-Validator repository. Defaults to sibling folder FSMP-Validator.
.PARAMETER OutputDir
    Where to place the assembled package files. Defaults to "out/package".
.EXAMPLE
    .\build-local.ps1
.EXAMPLE
    .\build-local.ps1 -Variants @("", "-avx2")
#>
param(
    [string[]]$Variants   = @("", "-avx", "-avx2", "-avx512"),
    [string]$McmDir       = (Join-Path (Split-Path $PSScriptRoot -Parent) "FSMP-MCM"),
    [string]$ValidatorDir = (Join-Path (Split-Path $PSScriptRoot -Parent) "FSMP-Validator"),
    [string]$OutputDir    = (Join-Path $PSScriptRoot "out\package")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepoDir = $PSScriptRoot

function Write-Step { param([string]$msg) Write-Host "`n=== $msg ===" -ForegroundColor Cyan }
function Write-OK   { param([string]$msg) Write-Host "  OK: $msg" -ForegroundColor Green }
function Invoke-Cmd {
    param([string]$Exe, [string[]]$CmdArgs, [string]$WorkDir = $RepoDir)
    Write-Host "  > $Exe $($CmdArgs -join ' ')" -ForegroundColor DarkGray
    Push-Location $WorkDir
    try { & $Exe @CmdArgs; if ($LASTEXITCODE -ne 0) { throw "Command failed with exit code $LASTEXITCODE" } }
    finally { Pop-Location }
}

# ---------------------------------------------------------------------------
# 0. Set up B:\ short-path alias to stay under Windows MAX_PATH=260
#    The avx2/avx512 vcpkg triplet paths exceed 260 chars from the full repo path.
#    By mapping B:\ -> repo root, all generated project paths stay well under 260.
# ---------------------------------------------------------------------------
Write-Step "Setting up B:\\ short-path alias"
$SubstOutput = (& subst 2>&1) -join "`n"
if ($SubstOutput -notmatch 'B:\\') {
    & subst B: $RepoDir
    if ($LASTEXITCODE -ne 0) { throw "Failed to create subst B: -> $RepoDir" }
    Write-OK "Created subst B: -> $RepoDir"
} else {
    Write-OK "B:\\ already mapped"
}
if (-not (Test-Path "B:\CMakeLists.txt")) { throw "B:\\ not accessible or not mapped to repo root" }

# ---------------------------------------------------------------------------
# 1. Build all requested AVX variants (all run from B:\ for short paths)
# ---------------------------------------------------------------------------
$BuiltVariants = @()

# The noavx triplet's bullet3 package contains the header; avx2/avx512 binary cache entries
# are corrupted (missing btDiscreteCollisionDetectorInterface.h). Copy it after configure.
$NoavxBulletHeader = "B:\out\build\vs2026-windows\vcpkg_installed\x64-windows-static-md\include\bullet\BulletCollision\NarrowPhaseCollision\btDiscreteCollisionDetectorInterface.h"

foreach ($Suffix in $Variants) {
    $LocalPreset  = "vs2026-windows$Suffix"  # e.g. vs2026-windows-avx2
    $PackageName  = "raw-vs2022-windows$Suffix"  # e.g. raw-vs2022-windows-avx2
    $BuildDir     = Join-Path $RepoDir "out\build\$LocalPreset"
    $BuildDirB    = "B:\out\build\$LocalPreset"  # same dir via short-path alias
    $PluginsDir   = Join-Path $BuildDir "plugins\SKSE\Plugins"
    $TripletName  = "x64-windows-static-md$Suffix"

    Write-Step "Build variant: $LocalPreset  ->  $PackageName"

    # Ensure the post-build copy target directory exists BEFORE configure
    New-Item -ItemType Directory -Force -Path $PluginsDir | Out-Null

    # If existing CMakeCache.txt has C:\ paths, delete it so cmake regenerates using B:\
    $CacheFile = Join-Path $BuildDir "CMakeCache.txt"
    if (Test-Path $CacheFile) {
        $CacheContent = Get-Content $CacheFile -Raw
        if ($CacheContent -match "CMAKE_HOME_DIRECTORY:INTERNAL=C:") {
            Remove-Item $CacheFile -Force
            $CMakeFilesDir = Join-Path $BuildDir "CMakeFiles"
            if (Test-Path $CMakeFilesDir) { Remove-Item $CMakeFilesDir -Recurse -Force }
            Write-Host "  Deleted stale CMakeCache (C:\\ paths -> regenerating with B:\\ paths)" -ForegroundColor Yellow
        }
    }

    # Configure from B:\ so ${sourceDir}=B: keeps vcpkg include paths under 260 chars
    Invoke-Cmd cmake @("--preset", $LocalPreset) -WorkDir "B:\"

    # Fix corrupted vcpkg binary cache for avx2/avx512: bullet3 is missing one header
    if ($Suffix -in @("-avx2", "-avx512")) {
        $BulletHeaderDst = "$BuildDirB\vcpkg_installed\$TripletName\include\bullet\BulletCollision\NarrowPhaseCollision\btDiscreteCollisionDetectorInterface.h"
        if (-not (Test-Path $BulletHeaderDst)) {
            if (Test-Path $NoavxBulletHeader) {
                Copy-Item $NoavxBulletHeader $BulletHeaderDst -Force
                Write-OK "Copied missing bullet3 header for $TripletName"
            } else {
                Write-Warning "Cannot fix missing bullet3 header: source not found at $NoavxBulletHeader"
            }
        }
    }

    # Build from B:\ (uses short paths in MSBuild invocation)
    Invoke-Cmd cmake @("--build", $BuildDirB, "--config", "Release") -WorkDir "B:\"

    $DllPath = Join-Path $PluginsDir "hdtsmp64.dll"
    if (-not (Test-Path $DllPath)) { throw "DLL not found at $DllPath after building $LocalPreset" }
    Write-OK "DLL: $DllPath"

    $BuiltVariants += @{ LocalPreset = $LocalPreset; PackageName = $PackageName; BuildDir = $BuildDir }
}

# ---------------------------------------------------------------------------
# 2. Compile MCM Papyrus scripts (once, independent of AVX variant)
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
# 3. Assemble package
# ---------------------------------------------------------------------------
Write-Step "Assembling package -> $OutputDir"

if (Test-Path $OutputDir) { Remove-Item -Recurse -Force $OutputDir }
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

# 3a. fomod tree content -> root of package
Copy-Item -Path (Join-Path $RepoDir "fomod tree\*") -Destination $OutputDir -Recurse -Force
Write-OK "Fomod tree copied"

# 3b. DLL artifacts for each variant -> raw-vs2022-windows* (CI-compatible names)
foreach ($v in $BuiltVariants) {
    $RawDir = Join-Path $OutputDir $v.PackageName
    Copy-Item -Path (Join-Path $v.BuildDir "plugins") -Destination $RawDir -Recurse -Force
    Write-OK "Artifacts: $($v.LocalPreset) -> $($v.PackageName)"
}

# 3c. Validator files -> FSMPV/
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

# 3d. MCM files -> FSMPM/
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
# 4. Create ZIP archive
# ---------------------------------------------------------------------------
Write-Step "Creating ZIP archive"
$_7zCmd = Get-Command "7z.exe" -ErrorAction SilentlyContinue
$SevenZip = if ($_7zCmd) { $_7zCmd.Source } else { $null }
if (-not $SevenZip) {
    foreach ($p in @("C:\Program Files\7-Zip\7z.exe","C:\Program Files (x86)\7-Zip\7z.exe")) {
        if (Test-Path $p) { $SevenZip = $p; break }
    }
}

$BaseVersion = (Select-String -Path (Join-Path $RepoDir "CMakeLists.txt") -Pattern 'VERSION\s+(\d+\.\d+\.\d+)' | Select-Object -First 1).Matches.Groups[1].Value
$ShortSha    = (git -C $RepoDir rev-parse --short HEAD 2>$null) -replace "`n",""
$BranchRaw   = (git -C $RepoDir rev-parse --abbrev-ref HEAD 2>$null) -replace "`n",""
$Branch      = $BranchRaw -replace "/", "-"
$TagVersion  = if ($Branch -eq "master") { $BaseVersion } else { "$BaseVersion-$Branch-$ShortSha" }
$ZipName     = "FSMP-$TagVersion.zip"
$ZipPath     = Join-Path (Split-Path $OutputDir -Parent) $ZipName

if ($SevenZip) {
    if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
    Invoke-Cmd $SevenZip @("a", "-tzip", "-mm=LZMA", "-mx=9", $ZipPath, ".") -WorkDir $OutputDir
} else {
    Write-Warning "7z.exe not found - using Compress-Archive"
    if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
    Compress-Archive -Path (Join-Path $OutputDir "*") -DestinationPath $ZipPath
}

Write-Host "`n=== BUILD COMPLETE ===" -ForegroundColor Green
Write-Host "  Package : $ZipPath"
Write-Host "  Variants: $(($BuiltVariants | ForEach-Object { $_.PackageName }) -join ', ')"

