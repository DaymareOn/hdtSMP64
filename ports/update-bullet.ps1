# Script to update Bullet3 overlay port from vcpkg upstream
# Usage: .\ports\update-bullet.ps1

param(
    [string]$VcpkgRoot = "E:\Documents\source\repos\vcpkg"
)

Write-Host "Updating Bullet3 overlay port from vcpkg upstream..." -ForegroundColor Cyan

# Check if vcpkg root exists
if (-not (Test-Path $VcpkgRoot)) {
    Write-Host "Error: vcpkg root not found at $VcpkgRoot" -ForegroundColor Red
    Write-Host "Please specify correct path with -VcpkgRoot parameter" -ForegroundColor Yellow
    exit 1
}

$UpstreamPort = Join-Path $VcpkgRoot "ports\bullet3"
$OverlayPort = Join-Path (Split-Path $PSScriptRoot -Parent) "cmake\ports\bullet3"

# Check upstream port exists
if (-not (Test-Path $UpstreamPort)) {
    Write-Host "Error: Upstream bullet3 port not found at $UpstreamPort" -ForegroundColor Red
    exit 1
}

Write-Host "`nStep 1: Pulling latest vcpkg..." -ForegroundColor Yellow
Push-Location $VcpkgRoot
git pull
Pop-Location

Write-Host "`nStep 2: Checking version differences..." -ForegroundColor Yellow
$UpstreamVersion = (Get-Content (Join-Path $UpstreamPort "vcpkg.json") | ConvertFrom-Json).version
$OverlayVersion = (Get-Content (Join-Path $OverlayPort "vcpkg.json") | ConvertFrom-Json).version

Write-Host "  Upstream version: $UpstreamVersion" -ForegroundColor Gray
Write-Host "  Overlay version:  $OverlayVersion" -ForegroundColor Gray

if ($UpstreamVersion -ne $OverlayVersion) {
    Write-Host "`n  WARNING: Version mismatch detected!" -ForegroundColor Red
    Write-Host "  This may require updating your custom modifications." -ForegroundColor Yellow
}

Write-Host "`nStep 3: Backing up current overlay port..." -ForegroundColor Yellow
$BackupDir = Join-Path (Split-Path $PSScriptRoot -Parent) "cmake\ports\bullet3.backup.$(Get-Date -Format 'yyyyMMdd-HHmmss')"
Copy-Item -Path $OverlayPort -Destination $BackupDir -Recurse
Write-Host "  Backup created: $BackupDir" -ForegroundColor Gray

Write-Host "`nStep 4: Copying upstream files..." -ForegroundColor Yellow
# Copy vcpkg.json (contains version info)
Copy-Item (Join-Path $UpstreamPort "vcpkg.json") (Join-Path $OverlayPort "vcpkg.json") -Force
Write-Host "  Updated: vcpkg.json" -ForegroundColor Gray

# Copy usage file
Copy-Item (Join-Path $UpstreamPort "usage") (Join-Path $OverlayPort "usage") -Force
Write-Host "  Updated: usage" -ForegroundColor Gray

# Copy patch files (but don't overwrite our custom ones)
Get-ChildItem (Join-Path $UpstreamPort "*.diff") | ForEach-Object {
    Copy-Item $_.FullName (Join-Path $OverlayPort $_.Name) -Force
    Write-Host "  Updated: $($_.Name)" -ForegroundColor Gray
}

# Save new portfile for comparison
$NewPortfile = Join-Path $OverlayPort "portfile.cmake.new"
Copy-Item (Join-Path $UpstreamPort "portfile.cmake") $NewPortfile -Force
Write-Host "  Created: portfile.cmake.new (for manual review)" -ForegroundColor Gray

Write-Host "`nStep 5: Review required!" -ForegroundColor Yellow
Write-Host @"

  The upstream portfile.cmake has been saved as portfile.cmake.new

  You need to:
  1. Compare portfile.cmake with portfile.cmake.new
  2. Merge any important upstream changes
  3. Ensure your custom modification blocks are preserved
  4. Test the build

  Custom modification markers to preserve:
    # Apply non-Hookean spring modifications
    file(READ ...
    string(REPLACE ...

"@ -ForegroundColor Cyan

Write-Host "`nStep 6: Testing build (optional)..." -ForegroundColor Yellow
$response = Read-Host "Do you want to test the build now? (y/N)"
if ($response -eq 'y' -or $response -eq 'Y') {
    Write-Host "`n  Cleaning build cache..." -ForegroundColor Gray
    $BuildVcpkg = Join-Path (Split-Path $PSScriptRoot -Parent) "build\vcpkg_installed"
    if (Test-Path $BuildVcpkg) {
        Remove-Item -Recurse -Force $BuildVcpkg
    }

    Write-Host "  Running CMake configure..." -ForegroundColor Gray
    $ProjectRoot = Split-Path $PSScriptRoot -Parent
    cmake -S $ProjectRoot -B (Join-Path $ProjectRoot "build") `
        -DCMAKE_TOOLCHAIN_FILE="$VcpkgRoot\scripts\buildsystems\vcpkg.cmake" `
        -DVCPKG_OVERLAY_PORTS=(Split-Path $OverlayPort -Parent)

    if ($LASTEXITCODE -eq 0) {
        Write-Host "`n  SUCCESS: Build configuration successful!" -ForegroundColor Green
    } else {
        Write-Host "`n  FAILED: Build configuration failed. Check your modifications." -ForegroundColor Red
        Write-Host "  Restore backup if needed: $BackupDir" -ForegroundColor Yellow
    }
}

Write-Host "`nDone!" -ForegroundColor Green
