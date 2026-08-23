@echo off
setlocal EnableExtensions

for %%I in ("%~dp0.") do set "PROJECT_ROOT=%%~fI"
set "BUILD_DIR=%PROJECT_ROOT%\build"
if not defined MAYA_LOCATION set "MAYA_LOCATION=C:\Program Files\Autodesk\Maya2024"
if not defined MAYA_DEVKIT_ROOT set "MAYA_DEVKIT_ROOT=%PROJECT_ROOT%\..\devkit\maya2024.2\devkitBase"
set "DEVKIT_LOCATION=%MAYA_DEVKIT_ROOT%"
set "MSVC_TOOLSET_VERSION=14.38.33130"
set "WINDOWS_SDK_VERSION=10.0.26100.0"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"

echo [DirectionalRetopo] Validating the Maya 2024.2 build environment...

if not exist "%MAYA_LOCATION%\bin\maya.exe" (
    echo ERROR: Maya was not found at "%MAYA_LOCATION%".
    goto :failure
)

for /f "usebackq delims=" %%V in (`powershell.exe -NoProfile -Command "(Get-Item -LiteralPath '%MAYA_LOCATION%\bin\maya.exe').VersionInfo.ProductVersion"`) do set "MAYA_PRODUCT_VERSION=%%V"
if not defined MAYA_PRODUCT_VERSION (
    echo ERROR: Could not read the Maya executable version.
    goto :failure
)

echo %MAYA_PRODUCT_VERSION% | "%SystemRoot%\System32\findstr.exe" /b /c:"24.2." >nul
if errorlevel 1 (
    echo ERROR: Expected Maya 2024.2, but maya.exe reports %MAYA_PRODUCT_VERSION%.
    goto :failure
)

if not exist "%MAYA_DEVKIT_ROOT%\include\maya\MFnPlugin.h" (
    echo ERROR: Maya DevKit headers were not found under "%MAYA_DEVKIT_ROOT%".
    goto :failure
)

if not exist "%MAYA_DEVKIT_ROOT%\lib\OpenMaya.lib" (
    echo ERROR: Maya DevKit libraries were not found under "%MAYA_DEVKIT_ROOT%".
    goto :failure
)

"%SystemRoot%\System32\findstr.exe" /r /c:"^[ ]*#define[ ]*MAYA_API_VERSION[ ]*20240200" "%MAYA_DEVKIT_ROOT%\include\maya\MTypes.h" >nul
if errorlevel 1 (
    echo ERROR: The configured DevKit is not Maya 2024 Update 2 ^(API 20240200^).
    goto :failure
)

if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe was not found at "%VSWHERE%".
    echo Install or repair Visual Studio Community 2022.
    goto :failure
)

set "VS2022_ROOT="
set "VSWHERE_RESULT=%TEMP%\DirectionalRetopo_vswhere_%RANDOM%_%RANDOM%.txt"
"%VSWHERE%" -products Microsoft.VisualStudio.Product.Community -version "[17.0,18.0)" -requires Microsoft.VisualStudio.Workload.NativeDesktop Microsoft.VisualStudio.Component.VC.Tools.x86.x64 Microsoft.VisualStudio.Component.VC.CMake.Project -latest -property installationPath > "%VSWHERE_RESULT%"
if errorlevel 1 (
    del /q "%VSWHERE_RESULT%" >nul 2>nul
    echo ERROR: vswhere.exe could not query Visual Studio 2022.
    goto :failure
)
set /p "VS2022_ROOT=" < "%VSWHERE_RESULT%"
del /q "%VSWHERE_RESULT%" >nul 2>nul

if not defined VS2022_ROOT (
    echo ERROR: A complete Visual Studio Community 2022 C++ installation was not found.
    echo Required: Desktop development with C++, MSVC v143 x64/x86, and C++ CMake tools for Windows.
    goto :failure
)

set "VSDEVCMD=%VS2022_ROOT%\Common7\Tools\VsDevCmd.bat"
set "CMAKE_EXE=%VS2022_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "EXPECTED_CL=%VS2022_ROOT%\VC\Tools\MSVC\%MSVC_TOOLSET_VERSION%\bin\Hostx64\x64\cl.exe"
set "SDK_INCLUDE=%ProgramFiles(x86)%\Windows Kits\10\Include\%WINDOWS_SDK_VERSION%\um\Windows.h"
set "SDK_LIBRARY=%ProgramFiles(x86)%\Windows Kits\10\Lib\%WINDOWS_SDK_VERSION%\um\x64\kernel32.lib"

if not exist "%VSDEVCMD%" (
    echo ERROR: Visual Studio 2022 VsDevCmd.bat was not found.
    goto :failure
)

if not exist "%EXPECTED_CL%" (
    echo ERROR: Required MSVC v143 toolset version %MSVC_TOOLSET_VERSION% was not found.
    echo Add MSVC v143 C++ x64/x86 build tools using Visual Studio Installer.
    goto :failure
)

if not exist "%SDK_INCLUDE%" (
    echo ERROR: Windows SDK %WINDOWS_SDK_VERSION% x64 headers were not found.
    echo Add the Windows 11 SDK using Visual Studio Installer.
    goto :failure
)

if not exist "%SDK_LIBRARY%" (
    echo ERROR: Windows SDK %WINDOWS_SDK_VERSION% x64 headers/libraries were not found.
    echo Add the Windows 11 SDK using Visual Studio Installer.
    goto :failure
)

if not exist "%CMAKE_EXE%" (
    echo ERROR: Visual Studio's bundled CMake was not found.
    echo Add C++ CMake tools for Windows using Visual Studio Installer.
    goto :failure
)

call "%VSDEVCMD%" -no_logo -arch=x64 -host_arch=x64 -vcvars_ver=14.38
if errorlevel 1 (
    echo ERROR: Visual Studio 2022 developer environment initialization failed.
    goto :failure
)

if /i not "%VSCMD_ARG_TGT_ARCH%"=="x64" (
    echo ERROR: Visual Studio developer environment did not select x64.
    goto :failure
)

pushd "%PROJECT_ROOT%"
if errorlevel 1 goto :failure

echo [DirectionalRetopo] Maya: %MAYA_LOCATION% ^(%MAYA_PRODUCT_VERSION%^)
echo [DirectionalRetopo] DevKit: %MAYA_DEVKIT_ROOT% ^(API 20240200^)
echo [DirectionalRetopo] Visual Studio: %VS2022_ROOT%
echo [DirectionalRetopo] MSVC: v143, %MSVC_TOOLSET_VERSION%, x64
echo [DirectionalRetopo] Windows SDK: %WINDOWS_SDK_VERSION%
"%CMAKE_EXE%" --version
if errorlevel 1 goto :build_failure

"%CMAKE_EXE%" -S "%PROJECT_ROOT%" -B "%BUILD_DIR%" -G "Visual Studio 17 2022" -A x64 -T "v143,version=%MSVC_TOOLSET_VERSION%" "-DCMAKE_SYSTEM_VERSION=%WINDOWS_SDK_VERSION%" "-DMAYA_LOCATION:PATH=%MAYA_LOCATION%" "-DMAYA_DEVKIT_ROOT:PATH=%MAYA_DEVKIT_ROOT%"
if errorlevel 1 goto :build_failure

"%CMAKE_EXE%" --build "%BUILD_DIR%" --config Release --parallel
if errorlevel 1 goto :build_failure

if not exist "%BUILD_DIR%\Release\DirectionalRetopo.mll" (
    echo ERROR: Build returned success but DirectionalRetopo.mll was not generated.
    goto :build_failure
)

echo [DirectionalRetopo] BUILD SUCCEEDED
echo [DirectionalRetopo] Output: %BUILD_DIR%\Release\DirectionalRetopo.mll
popd
endlocal
exit /b 0

:build_failure
set "BUILD_EXIT_CODE=%ERRORLEVEL%"
if "%BUILD_EXIT_CODE%"=="0" set "BUILD_EXIT_CODE=1"
echo [DirectionalRetopo] BUILD FAILED with exit code %BUILD_EXIT_CODE%.
popd
endlocal & exit /b %BUILD_EXIT_CODE%

:failure
echo [DirectionalRetopo] Environment validation failed.
endlocal
exit /b 1
