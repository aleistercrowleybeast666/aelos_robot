@echo off
setlocal
pushd "%~dp0" || exit /b 1
set PYTHONUTF8=1
where python >nul 2>&1
if errorlevel 1 (
    py -3 -B -m r3_deployer %*
) else (
    python -B -m r3_deployer %*
)
set "R3_EXIT=%ERRORLEVEL%"
if "%~1"=="" if not "%R3_EXIT%"=="0" pause
popd
exit /b %R3_EXIT%
