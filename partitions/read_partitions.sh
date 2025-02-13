#/bin/bash

source ~/esp/v5.2/export.sh
cd storage

#parttool.py --port "/dev/ttyUSB0" read_partition --partition-name=storage --output "storage.bin"

cd ../calibration
parttool.py --port "/dev/ttyUSB0" read_partition --partition-name=calibration --output "calibrationStored.bin"