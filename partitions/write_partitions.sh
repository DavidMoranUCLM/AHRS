source ~/esp/v5.2/export.sh

cd calibration
rm calibration.bin
cat A_mag_cal.bin b_mag_cal.bin >> calibration.bin

parttool.py --port "/dev/ttyUSB0" write_partition --partition-name=calibration --input "calibration.bin"