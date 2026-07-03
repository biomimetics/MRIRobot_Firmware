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

# Build and install OpenOCD from the cloned source. The Makefile's just_flash
# target loads tcl/interface and tcl/target scripts straight out of this repo
# checkout, and those scripts use syntax (e.g. "adapter usb vid_pid") that is
# newer than the openocd package in apt (0.12.0 on Ubuntu), so a distro
# package won't work — we need a binary built from this exact tree.
echo "Building OpenOCD..."
cd "$OPENOCD_DIR"
if ! command -v openocd &>/dev/null || ! openocd --version 2>&1 | grep -q "0.12.0+dev"; then
    echo "Installing OpenOCD build dependencies via apt..."
    sudo apt install -y libusb-1.0-0-dev libtool autoconf automake pkg-config texinfo libhidapi-dev
    git submodule update --init --recursive
    ./bootstrap with-submodules
    ./configure --enable-stlink --enable-internal-jimtcl
    make -j"$(nproc)"
    sudo make install
    echo "OpenOCD built and installed successfully."
else
    echo "Matching OpenOCD build already installed. Skipping build."
fi
cd "$SCRIPT_DIR"

# Install udev rules so ST-Link USB access doesn't require sudo/root.
echo "Installing OpenOCD udev rules..."
UDEV_RULES_SRC="$OPENOCD_DIR/contrib/60-openocd.rules"
UDEV_RULES_DST="/etc/udev/rules.d/60-openocd.rules"
if [ -f "$UDEV_RULES_SRC" ]; then
    if [ ! -f "$UDEV_RULES_DST" ] || ! diff -q "$UDEV_RULES_SRC" "$UDEV_RULES_DST" &>/dev/null; then
        sudo cp "$UDEV_RULES_SRC" "$UDEV_RULES_DST"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "Installed udev rules. Unplug and replug the ST-Link/board for permissions to take effect."
    else
        echo "OpenOCD udev rules already installed. Skipping."
    fi
else
    echo "Warning: udev rules not found at $UDEV_RULES_SRC. Skipping."
fi

# Make sure the current user can access USB debug adapters (plugdev group).
if ! id -nG "$USER" | grep -qw plugdev; then
    echo "Adding $USER to the plugdev group for USB adapter access..."
    sudo usermod -aG plugdev "$USER"
    echo "Added $USER to plugdev. Log out and back in for this to take effect."
else
    echo "$USER is already in the plugdev group. Skipping."
fi

echo "=== Setup complete ==="
