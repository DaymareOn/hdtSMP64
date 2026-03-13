# Bullet3 Overlay Port - Custom Modifications

This is a vcpkg overlay port for Bullet Physics 3.25 with custom modifications for hdtSMP64.

## Custom Modifications

### Non-Hookean Spring Support

**Original modification:** Based on commit `5abc9e33` from https://github.com/DaymareOn/hdtSMP64

- **Author:** blake-fm
- **Date:** Jan 15, 2023
- **Commit message:** "feat: Non-Hookean springs & many config options"
- **Original paths:** `hdtSMP64/BulletDynamics/ConstraintSolver/`

**Changes applied to `btGeneric6DofSpring2Constraint`:**

- Added `setNonHookeanDamping(int index, btScalar factor)` - Variable damping based on position
- Added `setNonHookeanStiffness(int index, btScalar factor)` - Variable stiffness based on position
- Modified constraint solver to apply variable damping/stiffness based on distance from equilibrium

These modifications allow for more realistic cloth/soft body physics in Skyrim by simulating non-Hookean material properties.

## Updating Bullet3

When vcpkg's Bullet3 port is updated:

1. **Check Bullet version in vcpkg:**

   ```bash
   cd /path/to/vcpkg
   git pull
   cat ports/bullet3/vcpkg.json  # Check version
   ```

2. **Update overlay port:**

   ```bash
   # Copy new port files (except our custom modifications)
   cp vcpkg/ports/bullet3/vcpkg.json ports/bullet3/
   cp vcpkg/ports/bullet3/portfile.cmake ports/bullet3/portfile.cmake.new

   # Merge the new portfile with our modifications
   # Compare portfile.cmake.new with portfile.cmake
   # Re-apply the source modification blocks
   ```

3. **Test the build:**

   ```bash
   # Clean vcpkg build cache
   rm -rf build/vcpkg_installed

   # Reconfigure and build
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_OVERLAY_PORTS=ports
   cmake --build build --config Release
   ```

4. **If Bullet's source structure changed:**
   - Check `build/vcpkg-manifest-install.log` for errors
   - Update the `string(REPLACE ...)` patterns in `portfile.cmake` to match new code structure
   - Test compilation

## Why Use String Replacements Instead of Patches?

We tested converting to traditional patch files (`.patch`/`.diff`) but found that:

- **Patch files are fragile**: They require exact line number matches and break when Bullet's code structure changes
- **String replacements are flexible**: They find the code pattern regardless of line numbers
- **Easier to maintain**: When Bullet updates, you only need to adjust if the actual code being replaced changes

The `string(REPLACE ...)` approach in `portfile.cmake` is actually a **best practice** for small, targeted modifications that need to survive version updates.

## Files Modified

- `src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h`

  - Added member variables for non-Hookean damping/stiffness to `btRotationalLimitMotor2` and `btTranslationalLimitMotor2`
  - Added method declarations

- `src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp`
  - Implemented `setNonHookeanDamping()` and `setNonHookeanStiffness()` methods
  - Modified `get_limit_motor_info2()` to apply variable damping/stiffness

## Viewing Original Changes

To see the exact original modifications from the upstream repository:

```bash
cd /path/to/hdtSMP64
git show 5abc9e33 -- hdtSMP64/BulletDynamics/ConstraintSolver/
```

To generate a diff showing only the Bullet changes:

```bash
git diff 5abc9e3~1 5abc9e3 -- hdtSMP64/BulletDynamics/ConstraintSolver/
```
