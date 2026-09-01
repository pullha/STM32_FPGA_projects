$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
. (Join-Path $PSScriptRoot "stm32-env.ps1")

Push-Location $projectRoot
try {
    & $script:CUBE_CMAKE --preset Debug
    & $script:CUBE_CMAKE --build --preset Debug
}
finally {
    Pop-Location
}
