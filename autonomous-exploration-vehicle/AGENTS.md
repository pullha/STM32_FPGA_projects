# Project Notes

Use the repository scripts for STM32 build and flash tasks:

- Build: `powershell -ExecutionPolicy Bypass -File .\scripts\build.ps1`
- Flash with ST-LINK/SWD: `powershell -ExecutionPolicy Bypass -File .\scripts\flash.ps1`
- Build and flash: `powershell -ExecutionPolicy Bypass -File .\scripts\build-and-flash.ps1`

The scripts locate STM32 VS Code extension binaries, STM32 GNU tools, Ninja, and STM32 Programmer from the local STM32Cube bundle paths, then update `PATH` for the current process.
