# Local cppcheck script - mirrors CI configuration
# Usage: .\scripts\cppcheck.ps1

$projectRoot = Split-Path -Parent $PSScriptRoot

cppcheck --enable=warning,performance,portability --error-exitcode=0 `
    -j $env:NUMBER_OF_PROCESSORS `
    --suppress=missingIncludeSystem `
    --suppress=unknownMacro `
    --suppress=noExplicitConstructor `
    "--suppress=*:*Bullet*" `
    "--suppress=*:*LinearMath*" `
    -i "$projectRoot\hdtSMP64\BulletCollision" `
    -i "$projectRoot\hdtSMP64\BulletDynamics" `
    -i "$projectRoot\hdtSMP64\Bullet3Collision" `
    -i "$projectRoot\hdtSMP64\Bullet3Dynamics" `
    -i "$projectRoot\hdtSMP64\LinearMath" `
    "$projectRoot\hdtSMP64\*.cpp" `
    "$projectRoot\hdtSMP64\hdtSkinnedMesh\*.cpp"
