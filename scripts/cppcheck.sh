#!/bin/bash
# Local cppcheck script - mirrors CI configuration
# Usage: ./scripts/cppcheck.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cppcheck --enable=warning,performance,portability --error-exitcode=0 \
    -j "$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)" \
    --suppress=missingIncludeSystem \
    --suppress=unknownMacro \
    --suppress=noExplicitConstructor \
    --suppress="*:*Bullet*" \
    --suppress="*:*LinearMath*" \
    -i "$PROJECT_ROOT/hdtSMP64/BulletCollision" \
    -i "$PROJECT_ROOT/hdtSMP64/BulletDynamics" \
    -i "$PROJECT_ROOT/hdtSMP64/Bullet3Collision" \
    -i "$PROJECT_ROOT/hdtSMP64/Bullet3Dynamics" \
    -i "$PROJECT_ROOT/hdtSMP64/LinearMath" \
    "$PROJECT_ROOT/hdtSMP64/"*.cpp \
    "$PROJECT_ROOT/hdtSMP64/hdtSkinnedMesh/"*.cpp
