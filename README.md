Alora Sensor Board for ConnIO
=============================

## Requirements

### ESP IoT Development Framework

You must have ESP IoT Development Framework (IDF) installed on your system. See [this link](http://esp-idf.readthedocs.io/en/latest/get-started/index.html#setup-toolchain) for installation guidance.

### Arduino Framework for ESP32

Arduino Framework for ESP32 can be used together with ESP IDF as its component. You can install Arduino for ESP32 by downloading the zip from [this link](https://github.com/espressif/arduino-esp32/archive/master.zip) and extract it to the `components/` directory on this repository.


Or, you can use symbolic link if you already have it installed on your system. Here's a sample command for adding Arduino ESP32 as component. The command should be compatible with UNIX-like system.

```bash
ln -s /path/to/arduino-esp32 components/
```

you must replace `/path/to/arduino-esp32` with actual path to arduino-esp32 on your system.

## Uploading to your ConnIO

First, you have to clone this repository or download it as zip. Here's the command for cloning this repository:

```bash
git clone https://github.com/dycodex/Alora-ConnIO AloraConnIO
```

You may need to run `make menuconfig` and exit immediately to apply default configuration from newer version of ESP-IDF.

Then, you can compile the code by running
```
make -j4
```

replace `4` in `-j4` with the number of core of your processor. It means that the compilation process will be splitted into 2 or more jobs. In this case, it will be splitted to 4 jobs.

If everything is ok, you can upload the firmware by running:

```
make flash
```

You can also open the serial monitor by running:

```
make monitor
```

**All of these command in single line**
```
make -j4 flash monitor
```

This command will compile and upload the source code to your ConnIO, then open a serial monitor.


