#/bin/bash

timestamp=$(date +"%Y%m%d_%H%M%S")

source ~/esp/v5.2/export.sh
cd storage

parttool.py --port "/dev/ttyUSB0" --baud 921600 read_partition --partition-name=storage --output "storage_${timestamp}.bin"

cd ../calibration
parttool.py --port "/dev/ttyUSB0" --baud 921600 read_partition --partition-name=calibration --output "calibrationStored_${timestamp}.bin"
