$ErrorActionPreference = "Stop"

function Get-LatestDirectory {
    param(
        [Parameter(Mandatory = $true)]
        [string] $Path,
        [Parameter(Mandatory = $true)]
        [string] $Filter
    )

    $directory = Get-ChildItem -Path $Path -Directory -Filter $Filter -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1

    if (-not $directory) {
        throw "Could not find directory '$Filter' under '$Path'."
    }

    return $directory.FullName
}

function Add-ToPathIfExists {
    param(
        [Parameter(Mandatory = $true)]
        [string] $Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Required path does not exist: $Path"
    }

    $pathParts = $env:Path -split ';'
    if ($pathParts -notcontains $Path) {
        $env:Path = "$Path;$env:Path"
    }
}

$extensionsDir = Join-Path $env:USERPROFILE ".vscode\extensions"
$bundlesDir = Join-Path $env:LOCALAPPDATA "stm32cube\bundles"

$cubeCoreDir = Get-LatestDirectory -Path $extensionsDir -Filter "stmicroelectronics.stm32cube-ide-core-*-win32-x64"
$cubeCmakeExtDir = Get-LatestDirectory -Path $extensionsDir -Filter "stmicroelectronics.stm32cube-ide-build-cmake-*-win32-x64"
$gnuToolsRoot = Join-Path $bundlesDir "gnu-tools-for-stm32"
$ninjaRoot = Join-Path $bundlesDir "ninja"
$programmerRoot = Join-Path $bundlesDir "programmer"

$gnuToolsDir = Get-LatestDirectory -Path $gnuToolsRoot -Filter "*"
$ninjaDir = Get-LatestDirectory -Path $ninjaRoot -Filter "*"
$programmerDir = Get-LatestDirectory -Path $programmerRoot -Filter "*"

$script:STM32_CUBE_BIN = Join-Path $cubeCoreDir "resources\binaries\win32\x86_64"
$script:STM32_CUBE_CMAKE_BIN = Join-Path $cubeCmakeExtDir "resources\cube-cmake\win32\x86_64"
$script:STM32_GNU_TOOLS_BIN = Join-Path $gnuToolsDir "bin"
$script:STM32_NINJA_BIN = Join-Path $ninjaDir "bin"
$script:STM32_PROGRAMMER_BIN = Join-Path $programmerDir "bin"

Add-ToPathIfExists $script:STM32_CUBE_BIN
Add-ToPathIfExists $script:STM32_CUBE_CMAKE_BIN
Add-ToPathIfExists $script:STM32_GNU_TOOLS_BIN
Add-ToPathIfExists $script:STM32_NINJA_BIN
Add-ToPathIfExists $script:STM32_PROGRAMMER_BIN

$script:CUBE_CMAKE = Join-Path $script:STM32_CUBE_CMAKE_BIN "cube-cmake.exe"
$script:STM32_PROGRAMMER_CLI = Join-Path $script:STM32_PROGRAMMER_BIN "STM32_Programmer_CLI.exe"

if (-not (Test-Path -LiteralPath $script:CUBE_CMAKE)) {
    throw "cube-cmake.exe was not found at $script:CUBE_CMAKE"
}

if (-not (Test-Path -LiteralPath $script:STM32_PROGRAMMER_CLI)) {
    throw "STM32_Programmer_CLI.exe was not found at $script:STM32_PROGRAMMER_CLI"
}
