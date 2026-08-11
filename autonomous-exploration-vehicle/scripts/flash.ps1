$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
. (Join-Path $PSScriptRoot "stm32-env.ps1")

$elfPath = Join-Path $projectRoot "build\Debug\Autonomous_Exploration_Vehicle.elf"
if (-not (Test-Path -LiteralPath $elfPath)) {
    throw "ELF file not found. Run scripts\build.ps1 first."
}

& $script:STM32_PROGRAMMER_CLI -c port=SWD -w $elfPath -rst
