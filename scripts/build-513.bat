@echo off
set LOG=C:\Users\ricsz\source\repos\CuraFeather\conan_513_output.txt
echo Starting 5.13 build... > "%LOG%"

call "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64

echo VsDevCmd done, starting conan... >> "%LOG%"
cd /d "C:\Users\ricsz\source\repos\CuraFeather"

conan install . --build=missing -s build_type=Release >> "%LOG%" 2>&1
if errorlevel 1 ( echo CONAN_FAILED >> "%LOG%" && pause && exit /b 1 )

cmake --preset conan-release >> "%LOG%" 2>&1
if errorlevel 1 ( echo CMAKE_CONFIGURE_FAILED >> "%LOG%" && pause && exit /b 1 )

cmake --build --preset conan-release >> "%LOG%" 2>&1
if errorlevel 1 ( echo CMAKE_BUILD_FAILED >> "%LOG%" && pause && exit /b 1 )

echo BUILD_DONE >> "%LOG%"
echo Build complete! See conan_513_output.txt
pause
