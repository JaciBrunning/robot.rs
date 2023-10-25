git clone https://github.com/wpilibsuite/allwpilib.git

mv allwpilib/ntcore wpilib-hal/libs/ntcore
mv allwpilib/hal wpilib-hal/libs/hal
mv allwpilib/wpimath wpilib-hal/libs/wpimath
mv allwpilib/wpinet wpilib-hal/libs/wpinet
mv allwpilib/wpiutil wpilib-hal/libs/wpiutil

rm allwpilib -rf

cp wpilib-hal/libs/hal/src/main/native/include/* wpilib-hal/include -r
cp wpilib-hal/libs/ntcore/src/main/native/include/* wpilib-hal/include -r
cp wpilib-hal/libs/wpimath/src/main/native/include/* wpilib-hal/include -r
cp wpilib-hal/libs/wpinet/src/main/native/include/* wpilib-hal/include -r
cp wpilib-hal/libs/wpiutil/src/main/native/include/* wpilib-hal/include -r

