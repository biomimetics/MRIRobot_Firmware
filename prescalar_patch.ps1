# Patches the LF STM32F4 platform support to use PSC=83 (84MHz clock -> 1MHz TIM5 tick).
# Run this after setting up the lingua-franca repo, before rebuilding lfc-dev.

$Target = Join-Path $PSScriptRoot "resources\lingua-franca\core\src\main\resources\lib\c\reactor-c\low_level_platform\impl\src\lf_STM32f4_support.c"

if (-not (Test-Path $Target)) {
    Write-Host "ERROR: $Target not found. Run this script from the repo root."
    exit 1
}

(Get-Content -Raw $Target) -replace [regex]::Escape("TIM5->PSC = 15;"), "TIM5->PSC = 83;" | Set-Content -NoNewline $Target

if ((Get-Content -Raw $Target) -match [regex]::Escape("TIM5->PSC = 83;")) {
    Write-Host "Patch applied: TIM5->PSC set to 83 in $Target"
} else {
    Write-Host "ERROR: Patch failed. Check $Target manually."
    exit 1
}
