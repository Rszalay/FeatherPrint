@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64
cd /d C:\Users\ricsz\source\repos\CuraFeather\build\Release
echo Starting ninja build...
ninja
if errorlevel 1 (
    echo BUILD FAILED
) else (
    echo BUILD SUCCEEDED
)
pause
