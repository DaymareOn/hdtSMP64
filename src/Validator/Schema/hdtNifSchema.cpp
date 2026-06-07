#include "hdtNifSchema.h"

#include <pugixml.hpp>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Built-in SSE-focused nif.xml
// Covers the block types found in Skyrim SE (v20.2.0.7 / userVersion=12 /
// bsVersion=100) physics NIFs.  Version conditions have been pre-evaluated:
// fields that do not apply to SSE are simply omitted here.
// Unknown block types encountered at runtime cause the block walker to return
// nullopt, which triggers the existing heuristic fallback in the caller.
// ─────────────────────────────────────────────────────────────────────────────
static const char* kBuiltinSchema = R"XML(<?xml version="1.0" encoding="utf-8"?>
<niftoolsxml version="0.7.1.0">

  <!-- ── Basic types ── -->
  <basic name="bool"        count="1"/>
  <basic name="byte"        count="1"/>
  <basic name="ubyte"       count="1"/>
  <basic name="char"        count="1"/>
  <basic name="ushort"      count="2"/>
  <basic name="uint16"      count="2"/>
  <basic name="short"       count="2"/>
  <basic name="hfloat"      count="2"/>
  <basic name="int"         count="4"/>
  <basic name="int32"       count="4"/>
  <basic name="uint"        count="4"/>
  <basic name="uint32"      count="4"/>
  <basic name="ulittle32"   count="4"/>
  <basic name="float"       count="4"/>
  <basic name="uint64"      count="8"/>
  <basic name="StringIndex"  count="4"/>
  <basic name="NiFixedString" count="4"/>
  <basic name="Ref"            count="4" isref="true"/>
  <basic name="Ptr"            count="4" isref="true" isptr="true"/>
  <basic name="BlockTypeIndex" count="2"/>

  <!-- ── Compound types ── -->
  <compound name="NiPoint3">
    <add name="x" type="float"/> <add name="y" type="float"/> <add name="z" type="float"/>
  </compound>
  <compound name="Vector3">
    <add name="x" type="float"/> <add name="y" type="float"/> <add name="z" type="float"/>
  </compound>
  <compound name="Vector4">
    <add name="x" type="float"/> <add name="y" type="float"/>
    <add name="z" type="float"/> <add name="w" type="float"/>
  </compound>
  <compound name="NiMatrix3">
    <add name="m11" type="float"/> <add name="m12" type="float"/> <add name="m13" type="float"/>
    <add name="m21" type="float"/> <add name="m22" type="float"/> <add name="m23" type="float"/>
    <add name="m31" type="float"/> <add name="m32" type="float"/> <add name="m33" type="float"/>
  </compound>
  <compound name="NiQuaternion">
    <add name="w" type="float"/> <add name="x" type="float"/>
    <add name="y" type="float"/> <add name="z" type="float"/>
  </compound>
  <compound name="NiTransform">
    <add name="Rotation"    type="NiMatrix3"/>
    <add name="Translation" type="NiPoint3"/>
    <add name="Scale"       type="float"/>
  </compound>
  <compound name="NiBound">
    <add name="Center" type="NiPoint3"/> <add name="Radius" type="float"/>
  </compound>
  <compound name="Color3">
    <add name="r" type="float"/> <add name="g" type="float"/> <add name="b" type="float"/>
  </compound>
  <compound name="Color4">
    <add name="r" type="float"/> <add name="g" type="float"/>
    <add name="b" type="float"/> <add name="a" type="float"/>
  </compound>
  <compound name="hkVector4">
    <add name="x" type="float"/> <add name="y" type="float"/>
    <add name="z" type="float"/> <add name="w" type="float"/>
  </compound>
  <compound name="hkQuaternion">
    <add name="x" type="float"/> <add name="y" type="float"/>
    <add name="z" type="float"/> <add name="w" type="float"/>
  </compound>
  <compound name="hkMatrix3">
    <add name="c0" type="hkVector4"/> <add name="c1" type="hkVector4"/> <add name="c2" type="hkVector4"/>
  </compound>
  <compound name="Triangle">
    <add name="v1" type="ushort"/> <add name="v2" type="ushort"/> <add name="v3" type="ushort"/>
  </compound>
  <compound name="SkinTransform">
    <add name="Rotation"    type="NiMatrix3"/>
    <add name="Translation" type="NiPoint3"/>
    <add name="Scale"       type="float"/>
  </compound>
  <compound name="hkWorldObjCinfoProperty">
    <add name="data"             type="uint"/>
    <add name="size"             type="uint"/>
    <add name="capacityAndFlags" type="uint"/>
  </compound>
  <compound name="bhkCMSDMaterial">
    <add name="material" type="uint"/> <add name="filter" type="uint"/>
  </compound>
  <compound name="bhkCMSDBigTris">
    <add name="triangle1"   type="ushort"/> <add name="triangle2"   type="ushort"/>
    <add name="triangle3"   type="ushort"/> <add name="material"    type="uint"/>
    <add name="weldingInfo" type="ushort"/>
  </compound>
  <compound name="bhkCMSDTransform">
    <add name="translation" type="hkVector4"/> <add name="rotation" type="hkQuaternion"/>
  </compound>
  <compound name="bhkCMSDChunk">
    <add name="translation"    type="hkVector4"/>
    <add name="matIndex"       type="uint"/>    <add name="reference"     type="ushort"/>
    <add name="transformIndex" type="ushort"/>
    <add name="numVertices"    type="uint"/>
    <add name="vertices"       type="ushort" arr1="numVertices"/>
    <add name="numIndices"     type="uint"/>
    <add name="indices"        type="ushort" arr1="numIndices"/>
    <add name="numStrips"      type="uint"/>
    <add name="strips"         type="ushort" arr1="numStrips"/>
    <add name="numWeldingInfo" type="uint"/>
    <add name="weldingInfo"    type="ushort" arr1="numWeldingInfo"/>
  </compound>

  <!-- ── NiObject hierarchy ── -->
  <niobject name="NiObject" abstract="true"/>

  <niobject name="NiObjectNET" inherit="NiObject" abstract="true">
    <add name="Name"                type="StringIndex"/>
    <add name="Num Extra Data List" type="uint"/>
    <add name="Extra Data List"     type="Ref"  arr1="Num Extra Data List"/>
    <add name="Controller"          type="Ref"/>
  </niobject>

  <niobject name="NiAVObject" inherit="NiObjectNET" abstract="true">
    <add name="Flags"            type="uint"/>
    <add name="Translation"      type="NiPoint3"/>
    <add name="Rotation"         type="NiMatrix3"/>
    <add name="Scale"            type="float"/>
    <add name="Collision Object" type="Ref"/>
  </niobject>

  <niobject name="NiNode" inherit="NiAVObject">
    <add name="Num Children" type="uint"/>
    <add name="Children"     type="Ref" arr1="Num Children"/>
    <add name="Num Effects"  type="uint"/>
    <add name="Effects"      type="Ref" arr1="Num Effects"/>
  </niobject>

  <!-- NiNode subtypes -->
  <niobject name="BSFadeNode"     inherit="NiNode"/>
  <niobject name="BSLeafAnimNode" inherit="NiNode"/>
  <niobject name="BSDebrisNode"   inherit="NiNode"/>
  <niobject name="BSBlastNode"    inherit="NiNode"/>
  <niobject name="BSDamageStage"  inherit="NiNode"/>
  <niobject name="NiRoomGroup"    inherit="NiNode"/>
  <niobject name="BSOrderedNode"  inherit="NiNode">
    <add name="Alpha Sort Bound" type="NiPoint3"/>
    <add name="Static Bound"     type="bool"/>
  </niobject>
  <niobject name="BSMultiBoundNode" inherit="NiNode">
    <add name="Multi Bound"  type="Ref"/>
    <add name="Culling Type" type="uint"/>
  </niobject>
  <niobject name="NiSwitchNode" inherit="NiNode">
    <add name="Flags" type="ushort"/> <add name="Index" type="uint"/>
  </niobject>
  <niobject name="NiBillboardNode" inherit="NiNode">
    <add name="Billboard Mode" type="ushort"/>
  </niobject>
  <niobject name="BSValueNode" inherit="NiNode">
    <add name="Value" type="uint"/> <add name="Value Type" type="byte"/>
  </niobject>
  <niobject name="BSMasterParticleSystem" inherit="NiNode">
    <add name="Max Emitter Objects"  type="ushort"/>
    <add name="Num Particle Systems" type="uint"/>
    <add name="Particle Systems"     type="Ref" arr1="Num Particle Systems"/>
  </niobject>
  <niobject name="BSTreeNode" inherit="NiNode">
    <add name="Num Bones 1" type="uint"/>
    <add name="Bones 1"     type="Ref" arr1="Num Bones 1"/>
    <add name="Num Bones 2" type="uint"/>
    <add name="Bones 2"     type="Ref" arr1="Num Bones 2"/>
  </niobject>
  <niobject name="NiLODNode" inherit="NiNode">
    <add name="LOD Center"     type="NiPoint3"/>
    <add name="Num LOD Levels" type="uint"/>
    <!-- LOD ranges are floats, no refs -->
    <add name="LOD Data" type="float" arr1="Num LOD Levels"/>
    <add name="LOD Data2" type="float" arr1="Num LOD Levels"/>
  </niobject>
  <niobject name="NiMultiTargetTransformController" inherit="NiTimeController">
    <add name="Num Extra Target" type="ushort"/>
    <add name="Extra Targets"    type="Ref" arr1="Num Extra Target"/>
  </niobject>

  <!-- Legacy geometry -->
  <niobject name="NiGeometry" inherit="NiAVObject" abstract="true">
    <add name="Data"            type="Ref"/>
    <add name="Skin Instance"   type="Ref"/>
    <add name="Shader Property" type="Ref"/>
    <add name="Alpha Property"  type="Ref"/>
  </niobject>
  <niobject name="NiTriBasedGeom" inherit="NiGeometry" abstract="true"/>
  <niobject name="NiTriShape"     inherit="NiTriBasedGeom"/>
  <niobject name="NiTriStrips"    inherit="NiTriBasedGeom"/>

  <!-- SSE geometry -->
  <niobject name="BSTriShape" inherit="NiAVObject">
    <add name="Bounding Sphere"      type="NiBound"/>
    <add name="Skin"                 type="Ref"/>
    <add name="Shader Property"      type="Ref"/>
    <add name="Alpha Property"       type="Ref"/>
    <add name="Vertex Desc"          type="uint64"/>
    <add name="Num Triangles"        type="uint"/>
    <add name="Num Vertices"         type="ushort"/>
    <add name="Data Size"            type="uint"/>
    <add name="Vertex Triangle Data" type="byte" arr1="Data Size"/>
  </niobject>
  <niobject name="BSDynamicTriShape" inherit="BSTriShape">
    <add name="Dynamic Data Size" type="uint"/>
    <add name="Dynamic Data"      type="hkVector4" arr1="Num Vertices"/>
  </niobject>
  <niobject name="BSMeshLODTriShape" inherit="BSTriShape">
    <add name="LOD Level 0 Size" type="uint"/>
    <add name="LOD Level 1 Size" type="uint"/>
    <add name="LOD Level 2 Size" type="uint"/>
  </niobject>
  <niobject name="BSSubIndexTriShape" inherit="BSTriShape">
    <add name="Num Primitives"    type="uint"/>
    <add name="Num Segments"      type="uint"/>
    <add name="Total Num Prims"   type="uint"/>
    <add name="Segment Data"      type="uint" arr1="Num Segments"/>
    <add name="Sub Seg Data Size" type="uint"/>
    <add name="Sub Seg Data"      type="byte" arr1="Sub Seg Data Size"/>
  </niobject>
  <niobject name="BSCombinedTriShape" inherit="BSTriShape"/>

  <!-- Skinning -->
  <niobject name="NiSkinInstance" inherit="NiObject">
    <add name="Data"           type="Ref"/>
    <add name="Skin Partition" type="Ref"/>
    <add name="Skeleton Root"  type="Ref"/>
    <add name="Num Bones"      type="uint"/>
    <add name="Bones"          type="Ref" arr1="Num Bones"/>
  </niobject>
  <niobject name="BSDismemberSkinInstance" inherit="NiSkinInstance">
    <add name="Num Partitions" type="uint"/>
    <add name="Partitions"     type="uint" arr1="Num Partitions"/>
  </niobject>
  <niobject name="BSSkin::Instance" inherit="NiObject">
    <add name="Skeleton Root" type="Ref"/>
    <add name="Data"          type="Ref"/>
    <add name="Num Bones"     type="uint"/>
    <add name="Bones"         type="Ref" arr1="Num Bones"/>
    <add name="Num Scales"    type="uint"/>
    <add name="Scales"        type="float" arr1="Num Scales"/>
  </niobject>
  <!-- NiSkinData / NiSkinPartition / BSSkin::BoneData have no block refs -->
  <niobject name="NiSkinPartition" inherit="NiObject"/>
  <niobject name="NiSkinData"      inherit="NiObject"/>
  <niobject name="BSSkin::BoneData" inherit="NiObject"/>

  <!-- NiExtraData -->
  <niobject name="NiExtraData" inherit="NiObject" abstract="true">
    <add name="Name" type="StringIndex"/>
  </niobject>
  <niobject name="NiStringExtraData" inherit="NiExtraData">
    <add name="String Data" type="StringIndex"/>
  </niobject>
  <niobject name="NiBinaryExtraData" inherit="NiExtraData">
    <add name="Num Data" type="uint"/>
    <add name="Data"     type="byte" arr1="Num Data"/>
  </niobject>
  <niobject name="NiIntegerExtraData" inherit="NiExtraData">
    <add name="Integer Data" type="uint"/>
  </niobject>
  <niobject name="NiIntegersExtraData" inherit="NiExtraData">
    <add name="Num Integers" type="uint"/>
    <add name="Integers"     type="uint" arr1="Num Integers"/>
  </niobject>
  <niobject name="NiFloatExtraData" inherit="NiExtraData">
    <add name="Float Data" type="float"/>
  </niobject>
  <niobject name="NiFloatsExtraData" inherit="NiExtraData">
    <add name="Num Floats" type="uint"/>
    <add name="Floats"     type="float" arr1="Num Floats"/>
  </niobject>
  <niobject name="NiVectorExtraData" inherit="NiExtraData">
    <add name="Vector Data" type="Vector4"/>
  </niobject>
  <niobject name="BSXFlags"  inherit="NiIntegerExtraData"/>
  <niobject name="BSBound"   inherit="NiExtraData">
    <add name="Center"     type="NiPoint3"/>
    <add name="Dimensions" type="NiPoint3"/>
  </niobject>
  <niobject name="BSBehaviorGraphExtraData" inherit="NiExtraData">
    <add name="Behaviour Graph File"   type="StringIndex"/>
    <add name="Controls Base Skeleton" type="bool"/>
  </niobject>
  <niobject name="BSInvMarker" inherit="NiExtraData">
    <add name="Rotation X" type="ushort"/> <add name="Rotation Y" type="ushort"/>
    <add name="Rotation Z" type="ushort"/> <add name="Zoom"       type="float"/>
  </niobject>
  <niobject name="BSAnimNotes" inherit="NiExtraData">
    <add name="Num Notes" type="ushort"/>
    <add name="Notes"     type="uint"  arr1="Num Notes"/>
  </niobject>

  <!-- NiProperty -->
  <niobject name="NiProperty" inherit="NiObjectNET" abstract="true"/>
  <niobject name="NiAlphaProperty" inherit="NiProperty">
    <add name="Flags" type="ushort"/> <add name="Threshold" type="byte"/>
  </niobject>
  <niobject name="NiMaterialProperty" inherit="NiProperty">
    <add name="Specular Color" type="Color3"/> <add name="Emissive Color" type="Color3"/>
    <add name="Glossiness"     type="float"/>  <add name="Alpha"          type="float"/>
    <add name="Emissive Mult"  type="float"/>
  </niobject>
  <niobject name="NiStencilProperty" inherit="NiProperty">
    <add name="Enabled"          type="bool"/>  <add name="Fail Action"      type="uint"/>
    <add name="Z Fail Action"    type="uint"/>  <add name="Pass Action"      type="uint"/>
    <add name="Draw Mode"        type="uint"/>  <add name="Stencil Function" type="uint"/>
    <add name="Stencil Ref"      type="uint"/>  <add name="Stencil Mask"     type="uint"/>
  </niobject>
  <niobject name="NiVertexColorProperty" inherit="NiProperty">
    <add name="Flags" type="ushort"/>
  </niobject>
)XML"
									R"XML(  <niobject name="NiWireframeProperty" inherit="NiProperty">
    <add name="Flags" type="ushort"/>
  </niobject>
  <niobject name="NiZBufferProperty" inherit="NiProperty">
    <add name="Flags" type="ushort"/>
  </niobject>
  <niobject name="NiShadeProperty" inherit="NiProperty">
    <add name="Flags" type="ushort"/>
  </niobject>
  <niobject name="BSShaderProperty" inherit="NiProperty" abstract="true"/>
  <!-- BSLightingShaderProperty: NifSkope inserts Shader Type BEFORE Name in NiObjectNET via
       onlyT="BSLightingShaderProperty". Our parser does not support onlyT, so we break
       inheritance and list all fields explicitly with Shader Type first. -->
  <niobject name="BSLightingShaderProperty" inherit="">
    <add name="Shader Type"          type="uint"/>
    <add name="Name"                 type="StringIndex"/>
    <add name="Num Extra Data List"  type="uint"/>
    <add name="Extra Data List"      type="Ref" arr1="Num Extra Data List"/>
    <add name="Controller"           type="Ref"/>
    <add name="Shader Flags 1"       type="uint"/>
    <add name="Shader Flags 2"       type="uint"/>
    <add name="UV Offset U"          type="float"/>
    <add name="UV Offset V"          type="float"/>
    <add name="UV Scale U"           type="float"/>
    <add name="UV Scale V"           type="float"/>
    <add name="Texture Set"          type="Ref"/>
    <!-- Remaining fields are all floats/scalars; no more Refs so we stop here. -->
  </niobject>
  <niobject name="BSEffectShaderProperty" inherit="BSShaderProperty">
    <add name="Source Texture"        type="StringIndex"/>
    <add name="Texture Clamp Mode"    type="uint"/>
    <add name="Lighting Influence"    type="byte"/>
    <add name="Env Map Min LOD"       type="byte"/>
    <add name="Unknown Byte"          type="byte"/>
    <add name="Falloff Start Angle"   type="float"/>
    <add name="Falloff Stop Angle"    type="float"/>
    <add name="Falloff Start Opacity" type="float"/>
    <add name="Falloff Stop Opacity"  type="float"/>
    <add name="Emissive Color"        type="Color4"/>
    <add name="Emissive Mult"         type="float"/>
    <add name="Soft Falloff Depth"    type="float"/>
    <add name="Greyscale Texture"     type="StringIndex"/>
    <add name="Env Map Texture"       type="StringIndex"/>
    <add name="Normal Texture"        type="StringIndex"/>
    <add name="Env Mask Texture"      type="StringIndex"/>
    <add name="Env Map Scale"         type="float"/>
  </niobject>
  <niobject name="BSWaterShaderProperty" inherit="BSShaderProperty">
    <add name="Water Shader Flags" type="ushort"/>
    <add name="Water Direction"    type="byte"/>
    <add name="Unknown Short 2"    type="ushort"/>
  </niobject>
  <niobject name="BSSkyShaderProperty" inherit="BSShaderProperty">
    <add name="Source Texture"  type="StringIndex"/>
    <add name="Sky Object Type" type="uint"/>
  </niobject>
  <niobject name="BSShaderTextureSet" inherit="NiObject">
    <add name="Num Textures" type="uint"/>
    <add name="Textures"     type="StringIndex" arr1="Num Textures"/>
  </niobject>

  <!-- NiTimeController -->
  <niobject name="NiTimeController" inherit="NiObject" abstract="true">
    <add name="Next Controller"   type="Ref"/>
    <add name="Flags"             type="ushort"/>
    <add name="Frequency"         type="float"/>
    <add name="Phase"             type="float"/>
    <add name="Start Time"        type="float"/>
    <add name="Stop Time"         type="float"/>
    <add name="Controller Target" type="Ptr"/>
  </niobject>
  <niobject name="NiSingleInterpController" inherit="NiTimeController" abstract="true">
    <add name="Interpolator" type="Ref"/>
  </niobject>
  <niobject name="NiTransformController"  inherit="NiSingleInterpController"/>
  <niobject name="NiPoint3InterpController" inherit="NiSingleInterpController"/>
  <niobject name="NiBoolInterpController" inherit="NiSingleInterpController"/>
  <niobject name="NiFloatInterpController" inherit="NiSingleInterpController"/>
  <niobject name="NiVisController"        inherit="NiSingleInterpController"/>
  <niobject name="BSFrustumFOVController" inherit="NiTimeController">
    <add name="Interpolator" type="Ref"/>
  </niobject>
  <niobject name="BSLagBoneController" inherit="NiTimeController">
    <add name="Linear Velocity"  type="float"/>
    <add name="Linear Rotation"  type="float"/>
    <add name="Maximum Distance" type="float"/>
  </niobject>
  <niobject name="NiGeomMorpherController" inherit="NiTimeController">
    <add name="Interpolator"       type="Ref"/>
    <add name="Always Update"      type="bool"/>
    <add name="Num Interpolators"  type="uint"/>
    <add name="Interpolators"      type="Ref"   arr1="Num Interpolators"/>
    <add name="Weights"            type="float"  arr1="Num Interpolators"/>
  </niobject>
  <niobject name="NiControllerManager" inherit="NiTimeController">
    <add name="Cumulative"                type="bool"/>
    <add name="Num Controller Sequences"  type="uint"/>
    <add name="Controller Sequences"      type="Ref" arr1="Num Controller Sequences"/>
    <add name="Object Palette"            type="Ref"/>
  </niobject>
  <niobject name="NiPathController" inherit="NiTimeController">
    <add name="Flags"          type="ushort"/>
    <add name="Bank Dir"       type="int"/>
    <add name="Max Bank Angle" type="float"/>
    <add name="Smoothing"      type="float"/>
    <add name="Follow Axis"    type="short"/>
    <add name="Path Data"      type="Ref"/>
    <add name="Pct Data"       type="Ref"/>
  </niobject>
  <niobject name="NiLookAtController" inherit="NiTimeController">
    <add name="Unknown Short" type="ushort"/>
    <add name="Look At"       type="Ref"/>
  </niobject>

  <!-- NiInterpolator -->
  <niobject name="NiInterpolator" inherit="NiObject" abstract="true"/>
  <niobject name="NiBlendInterpolator" inherit="NiInterpolator" abstract="true">
    <add name="Flags"            type="ushort"/>
    <add name="Array Size"       type="byte"/>
    <add name="Weight Threshold" type="float"/>
  </niobject>
  <niobject name="NiBlendBoolInterpolator"      inherit="NiBlendInterpolator">
    <add name="Value" type="bool"/>
  </niobject>
  <niobject name="NiBlendFloatInterpolator"     inherit="NiBlendInterpolator">
    <add name="Value" type="float"/>
  </niobject>
  <niobject name="NiBlendPoint3Interpolator"    inherit="NiBlendInterpolator">
    <add name="Value" type="NiPoint3"/>
  </niobject>
  <niobject name="NiBlendTransformInterpolator" inherit="NiBlendInterpolator"/>
  <niobject name="NiKeyBasedInterpolator" inherit="NiInterpolator" abstract="true"/>
  <niobject name="NiTransformInterpolator" inherit="NiKeyBasedInterpolator">
    <add name="Translation"    type="NiPoint3"/>
    <add name="Rotation"       type="NiQuaternion"/>
    <add name="Scale"          type="float"/>
    <add name="Transform Data" type="Ref"/>
  </niobject>
  <niobject name="NiPoint3Interpolator" inherit="NiKeyBasedInterpolator">
    <add name="Point Value" type="NiPoint3"/> <add name="Data" type="Ref"/>
  </niobject>
  <niobject name="NiBoolInterpolator" inherit="NiKeyBasedInterpolator">
    <add name="Bool Value" type="bool"/> <add name="Data" type="Ref"/>
  </niobject>
  <niobject name="NiFloatInterpolator" inherit="NiKeyBasedInterpolator">
    <add name="Float Value" type="float"/> <add name="Data" type="Ref"/>
  </niobject>
  <niobject name="BSRotAccumTransfInterpolator" inherit="NiTransformInterpolator"/>

  <!-- Key/sequence data — complex layout, no block refs in them -->
  <niobject name="NiKeyframeData"  inherit="NiObject"/>
  <niobject name="NiTransformData" inherit="NiObject"/>
  <niobject name="NiPosData"       inherit="NiObject"/>
  <niobject name="NiBoolData"      inherit="NiObject"/>
  <niobject name="NiFloatData"     inherit="NiObject"/>
  <niobject name="NiStringPalette" inherit="NiObject">
    <add name="Palette Len" type="uint"/>
    <add name="Palette"     type="byte" arr1="Palette Len"/>
  </niobject>

  <!-- NiCollisionObject -->
  <niobject name="NiCollisionObject" inherit="NiObject" abstract="true">
    <add name="Target" type="Ref"/>
  </niobject>
  <niobject name="bhkCollisionObject" inherit="NiCollisionObject">
    <add name="Flags" type="ushort"/> <add name="Body" type="Ref"/>
  </niobject>
  <niobject name="bhkNPCollisionObject"    inherit="bhkCollisionObject"/>
  <niobject name="bhkPCollisionObject"     inherit="bhkCollisionObject"/>
  <niobject name="bhkSPCollisionObject"    inherit="bhkCollisionObject"/>
  <niobject name="bhkBlendCollisionObject" inherit="bhkCollisionObject">
    <add name="Heir Gain" type="float"/> <add name="Vel Gain" type="float"/>
  </niobject>

  <!-- Havok shape hierarchy -->
  <niobject name="bhkRefObject"    inherit="NiObject"/>
  <niobject name="bhkSerializable" inherit="bhkRefObject" abstract="true"/>
  <niobject name="bhkWorldObject"  inherit="bhkSerializable" abstract="true">
    <add name="Shape"          type="Ref"/>
    <add name="Collision Filter" type="hkWorldObjCinfoProperty"/>
    <add name="Broad Phase Type" type="byte"/>
    <add name="Cinfo Property"   type="hkWorldObjCinfoProperty"/>
  </niobject>
  <niobject name="bhkEntity" inherit="bhkWorldObject" abstract="true"/>
  <!-- bhkRigidBody has a very large and complex layout. Its refs point to bhkShape /
       bhkConstraint / bhkAction — never to NiNode.  Mark as empty to force heuristic
       fallback for the ref-update step; the heuristic is safe here because bogus NiNode
       indices never appear in bhkRigidBody data. -->
  <niobject name="bhkRigidBody"  inherit="bhkEntity"/>
  <niobject name="bhkRigidBodyT" inherit="bhkRigidBody"/>
)XML"
									R"XML(  <niobject name="bhkSimpleShapePhantom" inherit="bhkWorldObject"/>

  <niobject name="bhkShape"           inherit="bhkRefObject" abstract="true"/>
  <niobject name="bhkSphereRepShape"  inherit="bhkShape" abstract="true">
    <add name="Material" type="uint"/> <add name="Radius" type="float"/>
  </niobject>
  <niobject name="bhkSphereShape"     inherit="bhkSphereRepShape"/>
  <niobject name="bhkCapsuleShape"    inherit="bhkSphereRepShape">
    <add name="Unused 1"     type="uint"/>   <add name="Unused 2"     type="uint"/>
    <add name="First Point"  type="hkVector4"/>
    <add name="Second Point" type="hkVector4"/>
    <add name="Radius 1"     type="float"/>  <add name="Radius 2"     type="float"/>
  </niobject>
  <niobject name="bhkBoxShape" inherit="bhkSphereRepShape">
    <add name="Unused 1"   type="uint"/> <add name="Unused 2"   type="uint"/>
    <add name="Dimensions" type="hkVector4"/>
    <add name="Unused 3"   type="float"/>
  </niobject>
  <niobject name="bhkConvexShape"          inherit="bhkSphereRepShape" abstract="true"/>
  <niobject name="bhkConvexTransformShape" inherit="bhkShape">
    <add name="Shape"     type="Ref"/>
    <add name="Material"  type="uint"/>
    <add name="Radius"    type="float"/>
    <add name="Unused 1"  type="byte"/>   <add name="Unused 2"  type="byte"/>
    <add name="Unused 3"  type="byte"/>   <add name="Unused 4"  type="byte"/>
    <add name="Transform" type="hkVector4"/>
    <add name="Rot 2"     type="hkVector4"/> <add name="Rot 3" type="hkVector4"/>
    <add name="Rot 4"     type="hkVector4"/>
  </niobject>
  <niobject name="bhkConvexVerticesShape" inherit="bhkConvexShape">
    <add name="Vertices Type"  type="hkWorldObjCinfoProperty"/>
    <add name="Normals Type"   type="hkWorldObjCinfoProperty"/>
    <add name="Num Vertices"   type="uint"/>
    <add name="Vertices"       type="hkVector4" arr1="Num Vertices"/>
    <add name="Num Normals"    type="uint"/>
    <add name="Normals"        type="hkVector4" arr1="Num Normals"/>
  </niobject>
  <niobject name="bhkMultiSphereShape" inherit="bhkSphereRepShape">
    <add name="Unknown Float 1" type="float"/> <add name="Unknown Float 2" type="float"/>
    <add name="Num Spheres"     type="uint"/>
    <add name="Spheres"         type="NiBound" arr1="Num Spheres"/>
  </niobject>
  <niobject name="bhkBvTreeShape" inherit="bhkShape" abstract="true"/>
  <niobject name="bhkMoppBvTreeShape" inherit="bhkBvTreeShape">
    <add name="Shape"          type="Ref"/>
    <add name="User Data"      type="uint"/>
    <add name="Unused 1"       type="byte"/>  <add name="Unused 2" type="byte"/>
    <add name="Unused 3"       type="byte"/>  <add name="Unused 4" type="byte"/>
    <add name="Build Type"     type="byte"/>
    <add name="MOPP Origin"    type="hkVector4"/>
    <add name="MOPP Scale"     type="float"/>
    <add name="MOPP Data Size" type="uint"/>
    <add name="MOPP Data"      type="byte" arr1="MOPP Data Size"/>
  </niobject>
  <niobject name="bhkShapeCollection" inherit="bhkShape" abstract="true"/>
  <niobject name="bhkListShape" inherit="bhkShapeCollection">
    <add name="Num Sub Shapes"   type="uint"/>
    <add name="Sub Shapes"       type="Ref" arr1="Num Sub Shapes"/>
    <add name="User Data"        type="hkWorldObjCinfoProperty"/>
    <add name="Num Unknown Ints" type="uint"/>
    <add name="Unknown Ints"     type="uint" arr1="Num Unknown Ints"/>
  </niobject>
  <niobject name="bhkCompressedMeshShape" inherit="bhkShape">
    <add name="Target"      type="Ref"/>
    <add name="User Data"   type="uint"/>
    <add name="Radius"      type="float"/>
    <add name="Radius Copy" type="float"/>
    <add name="Scale"       type="hkVector4"/>
    <add name="Data"        type="Ref"/>
  </niobject>
  <niobject name="bhkCompressedMeshShapeData" inherit="bhkRefObject">
    <add name="Bits Per Index"       type="uint"/>
    <add name="Bits Per W Index"     type="uint"/>
    <add name="Mask W Index"         type="uint"/>
    <add name="Mask Index"           type="uint"/>
    <add name="Error"                type="float"/>
    <add name="Aabb Half Extents"    type="hkVector4"/>
    <add name="Aabb Center"          type="hkVector4"/>
    <add name="Welding Type"         type="byte"/>
    <add name="Num Materials Old"    type="uint"/>
    <add name="Materials Old"        type="uint" arr1="Num Materials Old"/>
    <add name="Num Materials"        type="uint"/>
    <add name="Materials"            type="bhkCMSDMaterial" arr1="Num Materials"/>
    <add name="Num Named Materials"  type="uint"/>
    <add name="Named Materials"      type="uint" arr1="Num Named Materials"/>
    <add name="Num Transforms"       type="uint"/>
    <add name="Transforms"           type="bhkCMSDTransform" arr1="Num Transforms"/>
    <add name="Num Big Verts"        type="uint"/>
    <add name="Big Verts"            type="hkVector4" arr1="Num Big Verts"/>
    <add name="Num Big Tris"         type="uint"/>
    <add name="Big Tris"             type="bhkCMSDBigTris" arr1="Num Big Tris"/>
    <add name="Num Chunks"           type="uint"/>
    <add name="Chunks"               type="bhkCMSDChunk" arr1="Num Chunks"/>
    <add name="Num Convex Piece A"   type="uint"/>
  </niobject>

  <niobject name="bhkConstraint" inherit="bhkRefObject" abstract="true">
    <add name="Num Entities" type="uint"/>
    <add name="Entities"     type="Ref" arr1="Num Entities"/>
    <add name="Priority"     type="uint"/>
  </niobject>
  <!-- Constraint subtypes: complex pivot/axis data, no additional refs beyond parent -->
  <niobject name="bhkLimitedHingeConstraint"  inherit="bhkConstraint"/>
  <niobject name="bhkHingeConstraint"         inherit="bhkConstraint"/>
  <niobject name="bhkBallAndSocketConstraint" inherit="bhkConstraint"/>
  <niobject name="bhkStiffSpringConstraint"   inherit="bhkConstraint"/>
  <niobject name="bhkRagdollConstraint"       inherit="bhkConstraint"/>
  <niobject name="bhkWheelConstraint"         inherit="bhkConstraint"/>
  <niobject name="bhkPrismaticConstraint"     inherit="bhkConstraint"/>
  <niobject name="bhkMalleableConstraint"     inherit="bhkConstraint"/>
  <niobject name="bhkBreakableConstraint"     inherit="bhkConstraint"/>

  <!-- BSMultiBound -->
  <niobject name="BSMultiBound"     inherit="NiObject">
    <add name="Data" type="Ref"/>
  </niobject>
  <niobject name="BSMultiBoundData"   inherit="NiObject" abstract="true"/>
  <niobject name="BSMultiBoundOBB"    inherit="BSMultiBoundData">
    <add name="Center"   type="NiPoint3"/>
    <add name="Size"     type="NiPoint3"/>
    <add name="Rotation" type="NiMatrix3"/>
  </niobject>
  <niobject name="BSMultiBoundAABB"   inherit="BSMultiBoundData">
    <add name="Position" type="NiPoint3"/> <add name="Extent" type="NiPoint3"/>
  </niobject>
  <niobject name="BSMultiBoundSphere" inherit="BSMultiBoundData">
    <add name="Center" type="NiPoint3"/> <add name="Radius" type="float"/>
  </niobject>

  <niobject name="NiDefaultAVObjectPalette" inherit="NiObject">
    <add name="Scene"       type="Ref"/>
    <add name="Num Objects" type="uint"/>
    <!-- Each object is a StringIndex + Ref pair; simplified to just bytes here -->
    <add name="Object Data" type="byte" arr1="Num Objects"/>  <!-- placeholder, walker will fail -->
  </niobject>

</niftoolsxml>
)XML";

namespace hdt
{

	const char* getBuiltinNifSchemaXml() { return kBuiltinSchema; }

	const NifSchema& globalNifSchema()
	{
		static NifSchema s_schema;
		static std::once_flag s_flag;
		std::call_once(s_flag, [] { s_schema.loadForSSE(); });
		return s_schema;
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Version parsing helpers
	// ─────────────────────────────────────────────────────────────────────────────

	uint32_t NifSchema::parseVerStr(const std::string& v)
	{
		if (v.empty())
			return 0;
		uint32_t a = 0, b = 0, c = 0, d = 0;
		if (std::sscanf(v.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) < 1)
			return 0;
		return (a << 24) | (b << 16) | (c << 8) | d;
	}

	bool NifSchema::versionOk(
		const std::string& ver1, const std::string& ver2,
		const std::string& uver1, const std::string& uver2,
		const std::string& bsver1, const std::string& bsver2) const
	{
		if (!ver1.empty() && m_ver.version < parseVerStr(ver1))
			return false;
		if (!ver2.empty() && m_ver.version > parseVerStr(ver2))
			return false;
		if (!uver1.empty() && m_ver.userVersion < std::stoul(uver1))
			return false;
		if (!uver2.empty() && m_ver.userVersion > std::stoul(uver2))
			return false;
		if (!bsver1.empty() && m_ver.bsVersion < std::stoul(bsver1))
			return false;
		if (!bsver2.empty() && m_ver.bsVersion > std::stoul(bsver2))
			return false;
		return true;
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Schema loading
	// ─────────────────────────────────────────────────────────────────────────────

	bool NifSchema::loadFromString(const char* xml, const NifSchemaVersion& ver)
	{
		m_ver = ver;
		m_basics.clear();
		m_types.clear();

		pugi::xml_document doc;
		auto res = doc.load_string(xml);
		if (!res)
			return false;

		auto root = doc.child("niftoolsxml");
		if (!root)
			return false;

		// ── 1. Basic types ───────────────────────────────────────────────────────
		for (auto node : root.children("basic")) {
			std::string name = node.attribute("name").as_string();
			size_t count = static_cast<size_t>(node.attribute("count").as_uint(0));
			bool isref = node.attribute("isref").as_bool(false);
			bool isptr = node.attribute("isptr").as_bool(false);
			if (name.empty() || count == 0)
				continue;
			m_basics[name] = { count, isref, isptr };
		}

		// ── 2. Enum / bitfield types (treated as their storage primitive) ────────
		auto registerStorage = [&](pugi::xml_node n) {
			std::string name = n.attribute("name").as_string();
			std::string storage = n.attribute("storage").as_string("uint");
			if (name.empty())
				return;
			auto it = m_basics.find(storage);
			if (it != m_basics.end())
				m_basics[name] = { it->second.size, false };
		};
		for (auto node : root.children("enum")) registerStorage(node);
		for (auto node : root.children("bitfield")) registerStorage(node);

		// ── 3. Compound and niobject types ───────────────────────────────────────
		auto parseType = [&](pugi::xml_node node, bool isBlock) {
			std::string name = node.attribute("name").as_string();
			std::string inherit = node.attribute("inherit").as_string();
			if (name.empty())
				return;

			NifTypeDef def;
			def.name = name;
			def.inherit = inherit;
			def.isBlock = isBlock;

			for (auto add : node.children("add")) {
				// Pre-evaluate version conditions
				if (!versionOk(
						add.attribute("ver1").as_string(),
						add.attribute("ver2").as_string(),
						add.attribute("userver").as_string(),
						add.attribute("userver2").as_string(),
						add.attribute("bsver").as_string(),
						add.attribute("bsver2").as_string()))
					continue;

				NifFieldDef f;
				f.name = add.attribute("name").as_string();
				f.type = add.attribute("type").as_string();
				f.arr1 = add.attribute("arr1").as_string();
				f.cond = add.attribute("cond").as_string();
				if (f.name.empty() || f.type.empty())
					continue;
				def.fields.push_back(std::move(f));
			}
			m_types[name] = std::move(def);
		};

		for (auto node : root.children("compound")) parseType(node, false);
		for (auto node : root.children("niobject")) parseType(node, true);

		// ── 4. Compute hasAnyRefs for every type ─────────────────────────────────
		// A type has refs if any field is a Ref/Ptr basic, or any field's compound
		// type transitively has refs, or the inherited parent has refs.
		// Iterative fixed-point: safe even if inheritance chains are arbitrarily deep.
		{
			bool changed = true;
			while (changed) {
				changed = false;
				for (auto& [name, def] : m_types) {
					if (def.hasAnyRefs)
						continue;
					bool has = false;
					if (!def.inherit.empty()) {
						auto it = m_types.find(def.inherit);
						if (it != m_types.end() && it->second.hasAnyRefs)
							has = true;
					}
					if (!has) {
						for (const auto& field : def.fields) {
							auto bit = m_basics.find(field.type);
							if (bit != m_basics.end() && bit->second.isRef) {
								has = true;
								break;
							}
							auto tit = m_types.find(field.type);
							if (tit != m_types.end() && tit->second.hasAnyRefs) {
								has = true;
								break;
							}
						}
					}
					if (has) {
						def.hasAnyRefs = true;
						changed = true;
					}
				}
			}
		}

		return true;
	}

	bool NifSchema::loadForSSE(const std::string& externalXmlPath)
	{
		const NifSchemaVersion sseVer{ 0x14020007u, 12u, 100u };

		// Candidate paths to try, in priority order:
		//   1. Caller-supplied path
		//   2. NifSkope installations (gives full type coverage)
		//   3. Built-in SSE schema (always succeeds)
		static const char* kCandidates[] = {
			nullptr,  // placeholder for externalXmlPath
			"C:/Program Files/NifSkope 2.0 Dev 9/res/nif.xml",
			"C:/Program Files/NifSkope 2.0/res/nif.xml",
			"C:/Program Files (x86)/NifSkope/res/nif.xml",
			"C:/Program Files/NifSkope/res/nif.xml",
		};

		auto tryFile = [&](const std::string& path) -> bool {
			if (path.empty())
				return false;
			std::ifstream f(path, std::ios::binary | std::ios::ate);
			if (!f.is_open())
				return false;
			auto sz = f.tellg();
			if (sz <= 0 || sz > 64 * 1024 * 1024)
				return false;
			std::string buf(static_cast<size_t>(sz), '\0');
			f.seekg(0);
			f.read(buf.data(), sz);
			if (f.gcount() != static_cast<std::streamsize>(sz))
				return false;
			return loadFromString(buf.c_str(), sseVer);
		};

		if (tryFile(externalXmlPath))
			return true;
		for (size_t i = 1; i < std::size(kCandidates); ++i)
			if (tryFile(kCandidates[i]))
				return true;

		return loadFromString(kBuiltinSchema, sseVer);
	}

	const NifBasicType* NifSchema::findBasic(const std::string& name) const
	{
		auto it = m_basics.find(name);
		return it != m_basics.end() ? &it->second : nullptr;
	}

	const NifTypeDef* NifSchema::findType(const std::string& name) const
	{
		auto it = m_types.find(name);
		return it != m_types.end() ? &it->second : nullptr;
	}

	bool NifSchema::isKnown(const std::string& name) const
	{
		return m_basics.count(name) > 0 || m_types.count(name) > 0;
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// Block walker
	// ─────────────────────────────────────────────────────────────────────────────

	namespace
	{

		struct WalkCtx
		{
			const uint8_t* data;
			size_t dataSize;
			size_t pos = 0;
			int32_t numBlocks;
			bool ok = true;
			LinkFilter filter = LinkFilter::All;
			bool log = false;                      // emit per-field trace
			bool detail = false;                   // populate refArr1 + scalarPos
			std::map<std::string, uint32_t> vals;  // count fields (value)
			std::vector<size_t> refs;
			// detail mode only:
			std::vector<std::string> refArr1;         // parallel to refs: arr1 name or ""
			std::map<std::string, size_t> scalarPos;  // scalar field name → byte offset
		};

		bool evalCond(const std::string& cond, const WalkCtx& ctx)
		{
			if (cond.empty())
				return true;
			// Strip outer whitespace
			const char* p = cond.c_str();
			while (*p == ' ') ++p;

			// Pattern: "fieldName", "fieldName != N", "fieldName == N", "fieldName > N"
			// Find operator position
			const char* op = nullptr;
			int opType = 0;  // 1==, 2!=, 3>, 4<
			const char* q = p;
			while (*q && *q != '=' && *q != '!' && *q != '>' && *q != '<') ++q;
			if (*q == '=' && *(q + 1) == '=') {
				op = q;
				opType = 1;
			} else if (*q == '!' && *(q + 1) == '=') {
				op = q;
				opType = 2;
			} else if (*q == '>') {
				op = q;
				opType = 3;
			} else if (*q == '<') {
				op = q;
				opType = 4;
			}

			if (!op) {
				// Just a field name — truthy check
				auto it = ctx.vals.find(std::string(p));
				return it != ctx.vals.end() && it->second != 0;
			}

			std::string field(p, op - p);
			while (!field.empty() && field.back() == ' ') field.pop_back();
			const char* rhs = op + (opType == 1 || opType == 2 ? 2 : 1);
			while (*rhs == ' ') ++rhs;
			uint32_t rhsVal = static_cast<uint32_t>(std::strtoul(rhs, nullptr, 0));

			auto it = ctx.vals.find(field);
			uint32_t lhsVal = (it != ctx.vals.end()) ? it->second : 0;
			if (opType == 1)
				return lhsVal == rhsVal;
			if (opType == 2)
				return lhsVal != rhsVal;
			if (opType == 3)
				return lhsVal > rhsVal;
			if (opType == 4)
				return lhsVal < rhsVal;
			return true;
		}

		uint32_t resolveCount(const std::string& expr, const WalkCtx& ctx)
		{
			if (expr.empty())
				return 1;
			// Check for a bare integer literal
			bool allDigits = true;
			for (char c : expr)
				if (!std::isdigit(static_cast<unsigned char>(c))) {
					allDigits = false;
					break;
				}
			if (allDigits && !expr.empty())
				return static_cast<uint32_t>(std::stoul(expr));

			// Simple arithmetic: "FieldName / N" or "FieldName * N"
			auto slash = expr.find('/');
			auto star = expr.find('*');
			if (slash != std::string::npos || star != std::string::npos) {
				size_t opPos = (slash != std::string::npos) ? slash : star;
				std::string field = expr.substr(0, opPos);
				while (!field.empty() && field.back() == ' ') field.pop_back();
				uint32_t divisor = static_cast<uint32_t>(
					std::strtoul(expr.c_str() + opPos + 1, nullptr, 10));
				if (divisor == 0)
					divisor = 1;
				auto it = ctx.vals.find(field);
				uint32_t base = (it != ctx.vals.end()) ? it->second : 0;
				return (slash != std::string::npos) ? base / divisor : base * divisor;
			}

			// Plain field name lookup
			auto it = ctx.vals.find(expr);
			return (it != ctx.vals.end()) ? it->second : 0;
		}

		// Forward declarations
		void walkTypeName(const NifSchema& schema, const std::string& typeName, WalkCtx& ctx);
		void walkTypeDef(const NifSchema& schema, const NifTypeDef& def, WalkCtx& ctx);

		void walkTypeDef(const NifSchema& schema, const NifTypeDef& def, WalkCtx& ctx)
		{
			if (!ctx.ok)
				return;
			if (ctx.log)
				logger::info("[Walker]   walkTypeDef '{}' (inherit='{}') pos={}", def.name, def.inherit, ctx.pos);
			// Walk parent type first
			if (!def.inherit.empty()) {
				auto* parent = schema.findType(def.inherit);
				if (!parent) {
					if (ctx.log)
						logger::info("[Walker]     parent '{}' NOT FOUND → fail", def.inherit);
					ctx.ok = false;
					return;
				}
				walkTypeDef(schema, *parent, ctx);
			}
			// Walk own fields
			for (const auto& field : def.fields) {
				if (!ctx.ok)
					return;
				bool condOk = field.cond.empty() || evalCond(field.cond, ctx);
				if (ctx.log)
					logger::info("[Walker]     field '{}' type='{}' arr1='{}' cond='{}' condOk={}",
						field.name, field.type, field.arr1, field.cond, condOk);
				if (!condOk)
					continue;

				uint32_t count = field.arr1.empty() ? 1u : resolveCount(field.arr1, ctx);
				if (ctx.log && !field.arr1.empty())
					logger::info("[Walker]       count={} (from arr1='{}')", count, field.arr1);

				auto* basic = schema.findBasic(field.type);
				if (basic) {
					if (ctx.log)
						logger::info("[Walker]       basic type size={} isRef={} isPtr={}",
							basic->size, basic->isRef, basic->isPtr);
					// Non-ref large array: bulk skip
					if (count > 1 && !basic->isRef) {
						size_t total = static_cast<size_t>(count) * basic->size;
						if (ctx.pos + total > ctx.dataSize) {
							ctx.ok = false;
							return;
						}
						if (ctx.log)
							logger::info("[Walker]       bulk skip {} bytes (count={} × size={})",
								total, count, basic->size);
						ctx.pos += total;
						continue;
					}
					// Element-by-element (scalar or ref array)
					// Cap count for Ref arrays to what physically fits. A garbage arr1 value
					// (e.g. Num Children = 458751 in an unusual NIF) would otherwise exhaust
					// the block mid-array, set ctx.ok = false, and discard all refs already
					// collected. Non-ref scalars use the bounds check below as the hard stop.
					uint32_t effCount = count;
					if (basic->isRef && basic->size > 0 && !field.arr1.empty()) {
						uint32_t maxFit = (ctx.dataSize > ctx.pos) ? static_cast<uint32_t>((ctx.dataSize - ctx.pos) / basic->size) : 0u;
						effCount = std::min(effCount, maxFit);
					}
					for (uint32_t i = 0; i < effCount && ctx.ok; ++i) {
						if (ctx.pos + basic->size > ctx.dataSize) {
							ctx.ok = false;
							return;
						}
						if (basic->isRef) {
							bool collect = (ctx.filter == LinkFilter::All) || (ctx.filter == LinkFilter::RefOnly && !basic->isPtr) || (ctx.filter == LinkFilter::PtrOnly && basic->isPtr);
							int32_t rawVal = -1;
							std::memcpy(&rawVal, ctx.data + ctx.pos, 4);
							if (ctx.log)
								logger::info("[Walker]       Ref field '{}' [{}] off={} val={} isPtr={} filter={} collect={}",
									field.name, i, ctx.pos, rawVal, basic->isPtr,
									(ctx.filter == LinkFilter::All ? "All" : ctx.filter == LinkFilter::RefOnly ? "RefOnly" :
																												 "PtrOnly"),
									collect);
							if (collect) {
								ctx.refs.push_back(ctx.pos);
								if (ctx.detail)
									ctx.refArr1.push_back(field.arr1);
							}
						} else if (field.arr1.empty()) {
							// Store scalar value for later count resolution
							uint32_t v = 0;
							if (basic->size == 1)
								v = ctx.data[ctx.pos];
							else if (basic->size == 2) {
								uint16_t u;
								std::memcpy(&u, ctx.data + ctx.pos, 2);
								v = u;
							} else if (basic->size == 4)
								std::memcpy(&v, ctx.data + ctx.pos, 4);
							ctx.vals[field.name] = v;
							if (ctx.detail)
								ctx.scalarPos[field.name] = ctx.pos;
							if (ctx.log)
								logger::info("[Walker]       scalar '{}' = {} stored at pos={}", field.name, v, ctx.pos);
						}
						ctx.pos += basic->size;
					}
				} else {
					auto* compound = schema.findType(field.type);
					if (!compound) {
						if (ctx.log)
							logger::info("[Walker]       compound '{}' NOT FOUND → fail", field.type);
						ctx.ok = false;
						return;
					}
					if (ctx.log)
						logger::info("[Walker]       recurse into compound '{}'", field.type);
					for (uint32_t i = 0; i < count && ctx.ok; ++i)
						walkTypeDef(schema, *compound, ctx);
				}
			}
		}

		void walkTypeName(const NifSchema& schema, const std::string& typeName, WalkCtx& ctx)
		{
			if (!ctx.ok)
				return;
			auto* def = schema.findType(typeName);
			if (!def) {
				if (ctx.log)
					logger::info("[Walker] walkTypeName '{}' → NOT FOUND in schema → nullopt", typeName);
				ctx.ok = false;
				return;
			}
			if (ctx.log)
				logger::info("[Walker] walkTypeName '{}' → found", typeName);
			walkTypeDef(schema, *def, ctx);
		}

	}  // anonymous namespace

	std::optional<std::vector<size_t>> walkBlockRefs(
		const NifSchema& schema,
		const std::string& blockTypeName,
		const uint8_t* data,
		size_t dataSize,
		int32_t totalBlocks,
		LinkFilter filter,
		bool log)
	{
		// Short-circuit: if the type is known and has no Ref/Ptr fields anywhere in
		// its hierarchy, there is nothing to collect.  This avoids walking large
		// data-only blocks such as bhkCompressedMeshShapeData (1000+ chunks) that
		// account for the vast majority of remapAfterRemoval walker cost.
		{
			auto* def = schema.findType(blockTypeName);
			if (def && !def->hasAnyRefs)
				return std::vector<size_t>{};
		}

		WalkCtx ctx;
		ctx.data = data;
		ctx.dataSize = dataSize;
		ctx.filter = filter;
		ctx.numBlocks = totalBlocks;
		ctx.log = log;

		if (log)
			logger::info("[Walker] walkBlockRefs type='{}' filter={} dataSize={}",
				blockTypeName,
				(filter == LinkFilter::All ? "All" : filter == LinkFilter::RefOnly ? "RefOnly" :
																					 "PtrOnly"),
				dataSize);

		walkTypeName(schema, blockTypeName, ctx);

		// If the walk failed due to bounds overflow (e.g. a garbage Num Children count that
		// exhausts the block, or a trailing field like Num Effects that doesn't fit), but
		// refs were already collected before the overflow, return those partial results.
		// Returning nullopt would discard valid refs and cause getChildLinks to falsely
		// report no children, making bogus-node detection incorrectly remove live nodes.
		// True "unknown type" failures also set ctx.ok=false but produce no refs, so they
		// still return nullopt (the callers treat nullopt and empty-set the same way).
		if (!ctx.ok && ctx.refs.empty())
			return std::nullopt;

		// All offsets recorded by the walker are at schema-declared Ref fields.
		// Include them all: valid refs, null refs (-1), and anything in between.
		// The caller applies remap[v] which handles each case correctly.
		std::vector<size_t> result;
		result.reserve(ctx.refs.size());
		for (size_t off : ctx.refs) {
			if (off + 4 <= dataSize)
				result.push_back(off);
		}
		std::sort(result.begin(), result.end());
		result.erase(std::unique(result.begin(), result.end()), result.end());
		return result;
	}

	std::optional<BlockRefDetails> walkBlockRefDetails(
		const NifSchema& schema,
		const std::string& blockTypeName,
		const uint8_t* data,
		size_t dataSize,
		int32_t totalBlocks)
	{
		// Same short-circuit as walkBlockRefs: data-only types have no refs to remap.
		{
			auto* def = schema.findType(blockTypeName);
			if (def && !def->hasAnyRefs)
				return BlockRefDetails{};
		}

		WalkCtx ctx;
		ctx.data = data;
		ctx.dataSize = dataSize;
		ctx.filter = LinkFilter::All;
		ctx.numBlocks = totalBlocks;
		ctx.detail = true;

		walkTypeName(schema, blockTypeName, ctx);

		if (!ctx.ok && ctx.refs.empty())
			return std::nullopt;

		BlockRefDetails result;
		result.refs.reserve(ctx.refs.size());
		for (size_t i = 0; i < ctx.refs.size(); ++i) {
			size_t off = ctx.refs[i];
			if (off + 4 > dataSize)
				continue;
			BlockRefDetail d;
			d.offset = off;
			d.arr1 = (i < ctx.refArr1.size()) ? ctx.refArr1[i] : std::string{};
			result.refs.push_back(std::move(d));
		}
		result.countFieldPos = std::move(ctx.scalarPos);
		return result;
	}

}  // namespace hdt
