$ErrorActionPreference = "Stop"

& (Join-Path $PSScriptRoot "build.ps1")
& (Join-Path $PSScriptRoot "flash.ps1")
