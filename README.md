# hdtSMP with CUDA for Skyrim SE/AE/VR

Fork of [https://github.com/Karonar1/hdtSMP64] by Karonar1, of fork of [version](https://github.com/aers/hdtSMP64) by aers, from
[original code](https://github.com/HydrogensaysHDT/hdt-skyrimse-mods) by hydrogensaysHDT

## Changes

+ Supports Skyrim SE (1.5.97), VR (1.4.15), and AE (1.6.353, 1.6.640, 1.6.659, 1.6.1170).
+ Added maximum angle in ActorManager. A max angle can be used to specify physics on NPCs within a field of view.
  0 degrees represents straight in front of the camera. Default is 45 which is treated as + or - 45 degrees,
  so 90 total degrees. 180 would be all around.
+ Merged the CUDA version in the master branch through preprocessor directives to allow a continuous evolution,
  and easier building.
+ Added CUDA support for several parts of collision detection (still a work in progress). This includes
  everything that had OpenCL support in earlier releases, as well as the final collision check. CPU collision
  is still fully supported, and is used as a fallback if a CUDA-capable GPU is not available.
+ Added distance check in ActorManager to disable NPCs more than a certain distance from the player. This
  resolves the massive FPS drop in certain cell transitions (such as Blue Palace -> Solitude). Default
  maximum distance is 500.
+ New method for dynamically updating timesteps. This is technically less stable in terms of physics
  accuracy, but avoids a problem where one very slow frame (for example, where NPCs are added to the scene)
  causes a feedback loop that drops the framerate. If you saw framerates drop to 12-13 in the Blue Palace or
  at the Fire Festival, this should help.
+ Added can-collide-with-bone to mesh definitions, as a natural counterpart to no-collide-with-bone.
+ Added new "external" sharing option, for objects that should collide with other NPCs, but not this one.
  Good for defining the whole body as a collision mesh for interactions between characters.
+ Significant refactoring of armor handling in ActorManager, to be much stricter about disabling systems and
  reducing gradual FPS loss.
+ Changed prefix mechanism to use a simple incrementing value instead of trying to use pointers to objects.
  Previously this could lead to prefix collisions with a variety of weird effects and occasional crashes.
+ Skeletons should remain active as long as they are in the same cell as (and close enough to) the player
  character. Resolves an issue where entering the Ancestor Glade often incorrectly marked skeletons as
  inactive and disabled physics.
+ Added `smp list` console command to list tracked NPCs without so much detail - useful for checking which
  NPCs are active in crowded areas. NPCs are now sorted according to active status, with active ones last.
+ New mechanism for remapping mesh names in the defaultBBPs.xml file, allowing much more concise ways of
  defining complex collision objects for lots of armors at once.
+ The code to scan defaultBBPs.xml can now handle the structure of facegen files, which means head part
  physics should work (with limitations) on NPCs without having to manually edit the facegen data.
+ New bones from facegen files should now be added to the head instead of the NPC root, so they should be
  positioned correctly if there is no physics for them or after a reset.

## CUDA support

CUDA support is disabled by default, but can be enabled in configs.xml or from the console. It will
automatically fall back to the CPU algorithm if you do not have any CUDA capable cards. However, it does not
check capabilities of any cards it finds, so may crash if your card is too old. It was developed for a
GeForce 10 series card, so should work on those or anything newer.

The absolute minimum required compute capability is 3.5, to support dynamic parallelism. However, the plugin
is compiled for compute capability 5.0 for better performance with atomic operations. Therefore you will need
at least a Maxwell card to use the stock plugin, or a late model Kepler card if you compile it yourself.

If you have more than one CUDA-capable card, you can select the one used for physics calculations in the
configuration file. The code is designed to allow it to be changed at runtime, but there is currently no
console command to do this. By default it will use device 0, which is usually the most powerful card
available.

The following parts of the collision algortihm are currently GPU accelerated:

* Vertex position calculations for skinned mesh bodies
* Collider bounding box calculations for per-vertex and per-triangle shapes
* Aggregate bounding box calculation for internal nodes of collider trees
* Building collider lists for the final collision check
* Sphere-sphere and sphere-triangle collision checks
* Merging collision results (note that this may reduce performance for objects with lots of bones, as the
  merge buffer can get quite big - still working on this!)

The following parts are still CPU-based:

* Converting collision results to manifolds for the Bullet library to work with
* And, of course, the solver itself, which is part of the Bullet library, not the HDT-SMP plugin

This is still experimental, and may not give good performance. The old CPU collision algorithm was heavily
optimized, so matching its framerate is not easy. You will need a high end GPU and a low end CPU to see any
real performance benefits.

* On a 6850K processor (6 cores, 3.6GHz) with a 1080Ti GPU, framerate in crowded areas is a little worse
  than with the CPU-only algorithm. Most of the time, both algorithms easily reach the framerate cap at
  60fps.
* On the same hardware with only two cores enabled, the total collision time is around 2-3 times faster on
  GPU than CPU. Of course, this translates to a less impressive framerate difference, because collision
  checking is only one part of the HDT-SMP algorithm.

## Radeon support?

Nope, sorry. CUDA and nVidia cards are pretty much the industry standard for scientific computing, so that's
what I use. In any case, I can't support GPU architectures that I don't have. The plugin should work fine in
CPU mode with any type of card - I have tested it with Radeon 7850 and no nVidia drivers installed.

The plugin will use the most powerful available CUDA-capable card, regardless of whether it's being used for
anything else. In theory, it's possible to have a Radeon as the primary graphics card, and an nVidia card in
the same machine for CUDA and physics. I've tested this with two GeForce cards (a 1080Ti and a 1030) in the
same system, but not a Radeon and a GeForce.

## Note about NPC head parts

Head parts work fine for NPCs without valid facegen data, but this isn't very useful because it triggers the
infamous dark face bug. Special restrictions apply to NPCs that do have facegen data:

+ Only one XML file can be used per face, even if was built from multiple physics-enabled parts.
+ NiStringExtraData nodes aren't automatically copied into the facegen file, so physics won't work
  automatically. Either do it manually in NifSkope or map one of the head parts to a file in defaultBBPs.xml.
+ Bones that aren't explicitly referenced by any mesh are removed when facegen data is generated, and can't
  be used as kinematic objects in constraints. Replace references to these with the NPC head (which should
  always be present). You may also need to set the frame of reference origin to the position of the missing
  bone relative to the head to get correct constraint behavior.

## Console commands

The `smp` console command will print some basic information about the number of tracked and active objects.
The plugin recognizes the following optional parameters:

| Command | Description |
|---------|-------------|
| `smp reset` | Reloads configs.xml and attempts to reload all meshes and reset the physics system. May fail to reload some meshes or constraints properly. |
| `smp reload` | Alias for `smp reset`. |
| `smp on` | Enable physics simulation. |
| `smp off` | Disable physics simulation. |
| `smp gpu` | Toggle CUDA collision algorithm (CUDA builds only). Does nothing if no CUDA device available. |
| `smp timing [N]` | Start frame timing for N frames (default 200, range 10-1000). Results logged to hdtSMP64.log. |
| `smp metrics` | Toggle continuous performance logging to hdtSMP64.log. |
| `smp stats` | Display current performance statistics in the console. |
| `smp dumptree` | Dump the entire node tree of the targeted NPC to the log file. |
| `smp detail` | Show extended details of all tracked actors, including active/inactive armor and head parts. |
| `smp list` | Show a concise list of tracked actors, sorted by active status. |

## Coming soon (maybe)

+ Reworked tag system for better compartmentalized .xml files.
+ Continued work on CUDA algorithms.

## Known issues

+ Several options, including shape and collision definitions on bones, exist but don't seem to do anything.
+ Sphere-triangle collision check without penetration defined is obviously wrong, but fixing the test
  doesn't improve things. Needs further investigation.
+ Smp reset doesn't reload meshes correctly for some items (observed with Artesian Cloaks of Skyrim).
  Suspect references to the original triangle meshes are being dropped when they're no longer needed. We
  could keep ownership of the meshes, but it seems pretty marginal and a waste of memory. It also breaks
  some constraints for NPCs with facegen data.
+ It's not possible to refer to bones in facegen data that aren't used explicitly by at least one mesh. Most
  HDT hairs won't work as non-wig hair on NPCs without altering constraints. Probably possible but annoying
  to fix.
+ Physics enabled hair colours are sometimes wrong. This appears to happen there are two NPCs nearby using
  the same hair model.
+ Physics may be less stable due to the new dynamic timestep calculation.
+ Probably any open bug listed on Nexus that isn't resolved in changes, above. This list only contains
  issues I have personally observed.
+ [Known bugs on the Nexus](https://nexusmods.com/skyrimspecialedition/mods/57339?tab=bugs&BH=5)

## Build Instructions

Here is the [article](https://www.nexusmods.com/skyrimspecialedition/articles/3606) by DayDreamingDay.
These should be complete instructions to set up and build the plugin, without assuming prior coding
experience. I'm checking everything out into D:\Dev-noAVX and building without AVX support, but of course you
can use any directory.

You will need:
+ Visual Studio 2019 or newer (any edition)
+ Git
+ CMake
+ CUDA 11.6

### Build Configurations

Configuration names follow the pattern: `{VERSION}_{CUDA}_{AVX}[_DEBUG]`

**Game Versions:** `SE` (1.5.97), `VR` (1.4.15), `V1_6_353`, `V1_6_640`, `V1_6_659`, `V1_6_1170`

**CUDA Options:** `CUDA` (GPU collision) or `NOCUDA` (CPU-only)

**AVX Options:** `NoAVX` (max compatibility), `AVX`, `AVX2`, `AVX512`

Example: `V1_6_1170_NOCUDA_AVX2` builds for AE 1.6.1170 with CPU collision and AVX2 optimizations.

### Building Dependencies

Open a VS2019 command prompt ("x64 Native Tools Command Prompt for VS2019"). Download and build Detours and
Bullet:

```
d:
mkdir Dev-noAVX
cd Dev-noAVX
git clone https://github.com/microsoft/Detours.git
git clone https://github.com/bulletphysics/bullet3.git
cd Detours
nmake
cd ..\bullet3
cmake .
```

If you want AVX support in Bullet, use `cmake-gui` instead of `cmake`, and check the `USE_MSVC_AVX` box
before clicking Configure then Generate. This should give a fairly significant performance boost if your CPU
supports it.
+ Note that AVX support in Bullet and the HDT-SMP plugin itself are independent configuration options. Enable
  it in both for maximum performance; disable it in both for maximum compatibility.

Open D:\Dev-noAVX\bullet3\BULLET_PHYSICS.sln in Visual Studio, select the Release configuration, then
Build -> Build solution.

Download the appropriate SKSE64 source for your target game version and unpack into Dev-noAVX (source code
is included in the official distribution). For example, skse64_2_00_19 for SE 1.5.97, or a newer version
for AE builds. Then get the HDT-SMP source:

```
cd D:\Dev-noAVX\skse64_2_00_19\src\skse64
git init
git remote add origin https://github.com/Karonar1/hdtSMP64.git
git fetch
git checkout master
```

Open D:\Dev-noAVX\skse64_2_00_19\src\skse64\hdtSMP64.sln in Visual Studio. If you are asked to retarget
projects, just accept the defaults and click OK.

Open properties for the hdtSMP64 project. Select "All Configurations" at the top, and the C/C++ page. Add the
following to Additional Include Directories (just delete anything that was there before):
+ D:\Dev-noAVX\Detours\include
+ D:\Dev-noAVX\bullet3\src
+ D:\Dev-noAVX\skse64_2_00_19\src

On the Linker -> General page, add the following to Additional Library Directories:
+ D:\Dev-noAVX\bullet3\lib\Release
+ D:\Dev-noAVX\Detours\lib.X64

Open properties for the skse64 project. Select General, and change Configuration Type from "Dynamic Library
(.dll)" to "Static Library (.lib)".

## Credits

+ hydrogensaysHDT - Creating this plugin
+ aers - fixes and improvements
+ ousnius - some fixes, and "consulting"
+ karonar1 - bug fixes (the real work), I'm just building it
