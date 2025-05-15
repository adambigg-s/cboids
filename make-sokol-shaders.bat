@echo off

set arg1=%1
if "%arg1%"=="" set arg1=clean

sokol-shdc.exe -i ./src/shaders.glsl -o ./src/shaders.h --slang hlsl5:wgsl:glsl430

make %arg1%

