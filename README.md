# uNav_PID_Tuner
Software to tune the PID control on the uNav board

## Dependencies
Boost:
```shell
$ sudo apt-get install libboost-dev
```

Qt5:
```shell
$ sudo apt-get install qt5-default libqt5serialport5-dev
```

uDev:
```shell
$ sudo apt-get install libudev-dev
```

## Compile instructions
Clone the repository and the C++ interface library
```shell
$ mkdir ~/devel
$ mkdir ~/devel/uNavPidTuner
$ cd ~/devel/uNavPidTuner
$ git clone https://github.com/officinerobotiche/uNav_PID_Tuner.git
$ mkdir ~/devel/orblibcpp
$ cd ~/devel/orblibcpp
$ git clone https://github.com/officinerobotiche/orblibcpp.git
```
Now open QtCreator and open the project "uNav_PID_Tuner.pro" in the folder "~/devel/uNavPidTuner/uNav_PID_Tuner" 
