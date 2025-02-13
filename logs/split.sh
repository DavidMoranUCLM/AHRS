rm acc.txt gyro.txt mag.txt quat.txt
echo "ax,ay,az" >> acc.txt
echo "gx,gy,gz" >> gyro.txt
echo "mx,my,mz" >> mag.txt
echo "t,q0,qx,qy,qz" >> quat.txt

cat out.txt | grep "Accelerometer values" -a| awk '{match($0, /X: [-0-9.]+, Y: [-0-9.]+, Z: [-0-9.]+/, a); if (a[0]) {gsub(/[XYZ:]/, "", a[0]); gsub(/  /, "", a[0]); print a[0];}}' >> acc.txt
cat out.txt | grep "Angular velocity values" -a | awk '{match($0, /X: [-0-9.]+, Y: [-0-9.]+, Z: [-0-9.]+/, a); if (a[0]) {gsub(/[XYZ:]/, "", a[0]); gsub(/  /, "", a[0]); print a[0];}}' >> gyro.txt
cat out.txt | grep "Magnetometer values" -a| awk '{match($0, /X: [-0-9.]+, Y: [-0-9.]+, Z: [-0-9.]+/, a); if (a[0]) {gsub(/[XYZ:]/, "", a[0]); gsub(/  /, "", a[0]); print a[0];}}' >> mag.txt

cat out.txt | grep "PROCESOR_TASK" -a |  awk '{match($0, /t=[-0-9.]+, q0=[-0-9.]+, qx=[-0-9.]+, qy=[-0-9.]+, qz=[-0-9.]+/, a); if (a[0]) {gsub(/[tq0xyz=]/, "", a[0]); gsub(/  /, "", a[0]); print a[0];}}' >> quat.txt