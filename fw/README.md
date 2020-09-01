# Mini Clock Firmware

Firmware uses Espressif ESP-IDF. I've initially tried to get it working with PlatformIO, but was not successful as there are compatibility issues between cmake based IDF vs PlatformIO. Thus project is using IDF directly. I am on Linux (Debian), so the following steps are tested only there.

- Install IDF to your computer as described at [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
- [Configure your environment](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables). For example by `source ~/esp/esp-idf/export.sh`.
- Configure project with `idf.py menuconfig`. There is also a configuration section `Small Clock Configuration`.
- Build with `idf.py build` and flash with `idf.py flash`.
