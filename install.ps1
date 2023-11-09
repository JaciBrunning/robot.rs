git clone https:\\github.com\spacey-sooty\allwpilib.git

Move-Item allwpilib\ntcore wpilib-hal\libs\ntcore
Move-Item allwpilib\hal wpilib-hal\libs\hal
Move-Item allwpilib\wpimath wpilib-hal\libs\wpimath
Move-Item allwpilib\wpinet wpilib-hal\libs\wpinet
Move-Item allwpilib\wpiutil wpilib-hal\libs\wpiutil

Remove-Item allwpilib -Force -Recurse

Copy-Item wpilib-hal\libs\ntcore\src\main\native\include\* wpilib-hal\include
Copy-Item wpilib-hal\libs\hal\src\main\native\include\* wpilib-hal\include
Copy-Item wpilib-hal\libs\wpimath\src\main\native\include\* wpilib-hal\include
Copy-Item wpilib-hal\libs\wpinet\src\main\native\include\* wpilib-hal\include
Copy-Item wpilib-hal\libs\wpiutil\src\main\native\include\* wpilib-hal\include

