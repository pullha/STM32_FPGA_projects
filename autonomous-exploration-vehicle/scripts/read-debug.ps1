$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "stm32-env.ps1")

& $script:STM32_PROGRAMMER_CLI `
    -c port=SWD `
    -r32 0x200041c4 1 `
    -r32 0x200041c8 1 `
    -r32 0x200041cc 1 `
    -r32 0x200041d0 1 `
    -r32 0x200041d8 1 `
    -r32 0x200041dc 1 `
    -r32 0x200041e4 1 `
    -r32 0x20003d9c 1 `
    -r32 0x20003da0 1 `
    -r32 0x20003da4 1
