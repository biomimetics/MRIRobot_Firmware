$ErrorActionPreference = "Stop"

Write-Host "=== MRIRobot Firmware Setup Wizard ==="

$ScriptDir = $PSScriptRoot

# Clone and checkout the specific LF commit

$LfDir = Join-Path $ScriptDir "resources\lingua-franca"

Write-Host "Setting up Lingua Franca..."
if (-not (Test-Path $LfDir)) {
    Write-Host "Lingua Franca not found -- cloning into $LfDir..."
    git clone -b stm32_2 https://github.com/lf-lang/lingua-franca.git $LfDir
} else {
    Write-Host "Lingua Franca directory already exists at $LfDir. Skipping the clone..."
}
Set-Location $LfDir
Write-Host "Getting the right checkout..."
git checkout 60d9eacaa5ceb17587a3118e86d15e99b258b679
git submodule update --init --recursive
Write-Host "Applying prescalar patch for 84MHz clock..."
Set-Location $ScriptDir
& "$ScriptDir\prescalar_patch.ps1"
Set-Location $LfDir
Write-Host "Building Lingua Franca compiler (this may take a few minutes)..."
& "$LfDir\gradlew.bat" assemble

# Add lf binaries to PATH (user environment variable) if not already present
Write-Host "Checking if Lingua Franca binaries are in PATH..."
$LfBinPath = Join-Path $LfDir "bin"
$UserPath = [Environment]::GetEnvironmentVariable("Path", "User")
if (-not ($UserPath -split ";" -contains $LfBinPath)) {
    Write-Host "Lingua Franca bin not found in PATH! Adding it to the user PATH..."
    $NewPath = if ([string]::IsNullOrEmpty($UserPath)) { $LfBinPath } else { "$UserPath;$LfBinPath" }
    [Environment]::SetEnvironmentVariable("Path", $NewPath, "User")
    Write-Host "Added $LfBinPath to the user PATH. Open a new terminal for this to take effect there."
} else {
    Write-Host "Lingua Franca bin is already in the user PATH. No changes needed."
}
$env:Path = "$env:Path;$LfBinPath"

# Check ARM toolchain
Write-Host "Checking ARM toolchain..."
if (-not (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue)) {
    Write-Host "ARM toolchain not found! Please install the Arm GNU Toolchain manually:"
    Write-Host "  https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
    Write-Host "After installing, make sure its 'bin' directory is added to your PATH."
} else {
    Write-Host "ARM toolchain (arm-none-eabi-gcc) is already installed. Skipping installation."
}

# Clone openocd into resources if not already present
$OpenOcdDir = Join-Path $ScriptDir "resources\openocd"
Write-Host "Checking OpenOCD..."
if (-not (Test-Path $OpenOcdDir)) {
    Write-Host "OpenOCD not found! Cloning into $OpenOcdDir..."
    git clone https://github.com/openocd-org/openocd.git $OpenOcdDir
    Write-Host "OpenOCD cloned successfully."
} else {
    Write-Host "OpenOCD already present at $OpenOcdDir. Skipping clone."
}

Write-Host "=== Setup complete ==="
