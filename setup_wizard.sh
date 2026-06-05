#!/bin/bash
set -e

echo "=== MRIRobot Firmware Setup Wizard ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Clone and checkout the specific LF commit

LF_DIR="$SCRIPT_DIR/resources/lingua-franca"

echo "Setting up Lingua Franca..."
if [ ! -d "$LF_DIR" ]; then
    echo "Lingua Franca not found — cloning into $LF_DIR..."
    git clone -b stm32_2 https://github.com/lf-lang/lingua-franca.git "$LF_DIR"
else
    echo "Lingua Franca directory already exists at $LF_DIR. Skipping the clone..."
fi
cd "$LF_DIR"
echo "Getting the right checkout..."
git checkout 60d9eacaa5ceb17587a3118e86d15e99b258b679
git submodule update --init --recursive
echo "Applying prescalar patch for 84MHz clock..."
cd "$SCRIPT_DIR"
bash prescalar_patch.sh
cd "$LF_DIR"
echo "Building Lingua Franca compiler (this may take a few minutes)..."
./gradlew assemble

# Add lf binaries to PATH in .bashrc if not already present
echo "Checking if Lingua Franca binaries are in PATH..."
LF_BIN_PATH="$LF_DIR/bin"
LF_PATH_LINE="export PATH=\"\$PATH:$LF_BIN_PATH\""
if ! grep -qF "$LF_PATH_LINE" "$HOME/.bashrc"; then
    echo "Lingua Franca bin not found in PATH! Adding it to ~/.bashrc..."
    echo "$LF_PATH_LINE" >> "$HOME/.bashrc"
    echo "Added $LF_BIN_PATH to PATH in ~/.bashrc. Re-source your shell or start a new terminal for this to take effect."
else
    echo "Lingua Franca bin is already in PATH in ~/.bashrc. No changes needed."
fi
export PATH="$PATH:$LF_BIN_PATH"

# Install ARM toolchain
echo "Installing ARM toolchain..."
if ! command -v arm-none-eabi-gcc &>/dev/null; then
    echo "ARM toolchain not found! Installing gcc-arm-none-eabi via apt..."
    sudo apt install -y gcc-arm-none-eabi
    echo "ARM toolchain installed successfully."
else
    echo "ARM toolchain (arm-none-eabi-gcc) is already installed. Skipping installation."
fi

# Clone openocd into resources if not already present
OPENOCD_DIR="$SCRIPT_DIR/resources/openocd"
echo "Checking OpenOCD..."
if [ ! -d "$OPENOCD_DIR" ]; then
    echo "OpenOCD not found! Cloning into $OPENOCD_DIR..."
    git clone https://github.com/openocd-org/openocd.git "$OPENOCD_DIR"
    echo "OpenOCD cloned successfully."
else
    echo "OpenOCD already present at $OPENOCD_DIR. Skipping clone."
fi

echo "=== Setup complete ==="
