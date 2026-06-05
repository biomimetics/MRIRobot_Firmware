#!/bin/bash
# Patches the LF STM32F4 platform support to use PSC=83 (84MHz clock → 1MHz TIM5 tick).
# Run this after setting up the lingua-franca repo, before rebuilding lfc-dev.

TARGET="resources/lingua-franca/core/src/main/resources/lib/c/reactor-c/low_level_platform/impl/src/lf_STM32f4_support.c"

if [ ! -f "$TARGET" ]; then
    echo "ERROR: $TARGET not found. Run this script from the repo root."
    exit 1
fi

sed -i 's/TIM5->PSC = 15;/TIM5->PSC = 83;/' "$TARGET"

if grep -q "TIM5->PSC = 83;" "$TARGET"; then
    echo "Patch applied: TIM5->PSC set to 83 in $TARGET"
else
    echo "ERROR: Patch failed. Check $TARGET manually."
    exit 1
fi
