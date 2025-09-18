#!/bin/bash

# Script to test every possible compilation config
# - BME280_PROTOCOL: I2C or SPI
# - For I2C: BME280_SDO: LOW or HIGH
# - BME280_MODE: NORMAL or FORCED

# ANSI color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

PROJECT_DIR="$(pwd)"
MAKEFILE="$PROJECT_DIR/Makefile"  # Adjust if needed

configs=(
    "BME280_PROTOCOL=I2C BME280_SDO=LOW BME280_MODE=NORMAL"
    "BME280_PROTOCOL=I2C BME280_SDO=LOW BME280_MODE=FORCED"
    "BME280_PROTOCOL=I2C BME280_SDO=HIGH BME280_MODE=NORMAL"
    "BME280_PROTOCOL=I2C BME280_SDO=HIGH BME280_MODE=FORCED"
    "BME280_PROTOCOL=SPI BME280_MODE=NORMAL"
    "BME280_PROTOCOL=SPI BME280_MODE=FORCED"
)

echo -e "${YELLOW}Testing $(( ${#configs[@]} )) configurations...${NC}"

for i in "${!configs[@]}"; do
    config="${configs[$i]}"
    echo -e "=== ${YELLOW}Configuration $((i+1)) / ${#configs[@]}${NC} ==="
    echo -e "Variables: ${YELLOW}$config${NC}"
    
    # Clean previous build (silent, ignore errors on first run)
    make -f "$MAKEFILE" -s clean 2>/dev/null || echo "Clean skipped (may be first run)"
    
    # Compile with silenced output, capture warnings
    output=$(make -f "$MAKEFILE" -s $config all 2>&1)
    if [ $? -eq 0 ]; then
        # Count warnings (look for lines with "warning")
        warning_count=$(echo "$output" | grep -c "warning" || true)
        if [ $warning_count -gt 0 ]; then
            echo -e "${GREEN}✓ Compilation successful for config $((i+1)) with ${YELLOW}$warning_count warning(s)${NC}"
        else
            echo -e "${GREEN}✓ Compilation successful for config $((i+1)) with no warnings${NC}"
        fi
    else
        echo ""
        echo -e "${RED}=== FAILURE OUTPUT ===${NC}"
        echo "$output"  # Show error output for debugging
        echo -e "${RED}=== FAILURE OUTPUT ===${NC}"
        echo ""
        echo -e "${RED}✗ Compilation failed for config $((i+1)).${NC}\n${YELLOW}Config: $config${NC}"
        exit 1
    fi
    
    echo ""
done

echo -e "${GREEN}All configurations tested successfully!${NC}"
