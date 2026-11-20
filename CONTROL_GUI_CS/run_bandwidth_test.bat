@echo off
cd /d "%~dp0ControlGui.Wpf"
dotnet run -- --bandwidth
pause
