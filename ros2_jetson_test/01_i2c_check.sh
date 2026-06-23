#!/usr/bin/env bash
# T1 — Prove the CONTAINER can see the PCA9685 on I2C bus 1.
# Run INSIDE the ROS 2 container (started with --privileged -v /dev:/dev).
echo "==== T1 · I2C / PCA9685 visibility ===="
if ! command -v i2cdetect >/dev/null 2>&1; then
  echo "[*] i2c-tools not in image — installing (one time)..."
  apt-get update -qq && apt-get install -y -qq i2c-tools
fi
echo "[*] /dev/i2c-* visible to container:"; ls -1 /dev/i2c-* 2>/dev/null || echo "  NONE — container missing /dev access (check run flags)"
echo "[*] Scanning bus 1 (expect device at 0x40):"
i2cdetect -y -r 1
echo "---------------------------------------"
echo "PASS if '40' appears in the grid."
echo "(Bus 0 is dead on this hardware — only bus 1 matters.)"
