git clone https://github.com/wpilibsuite/allwpilib.git

Move-Item allwpilib\ntcore wpilib-hal\libs\ntcore
Move-Item allwpilib\hal wpilib-hal\libs\hal
Move-Item allwpilib\wpimath wpilib-hal\libs\wpimath
Move-Item allwpilib\wpinet wpilib-hal\libs\wpinet
Move-Item allwpilib\wpiutil wpilib-hal\libs\wpiutil

Remove-Item allwpilib -Force -Recurse

Copy-Item wpilib-hal\libs\ntcore\src\main\include wpilib-hal\include
Copy-Item wpilib-hal\libs\hal\src\main\include wpilib-hal\include
Copy-Item wpilib-hal\libs\wpimath\src\main\include wpilib-hal\include
Copy-Item wpilib-hal\libs\wpinet\src\main\include wpilib-hal\include
Copy-Item wpilib-hal\libs\wpiutil\src\main\include wpilib-hal\include

