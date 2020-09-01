# Mini Clock Firmware

Firmware uses Espressif ESP-IDF. I've initially tried to get it working with PlatformIO, but was not successful as there are compatibility issues between cmake based IDF vs PlatformIO. Thus project is using IDF directly. I am on Linux (Debian), so the following steps are tested only there.

- Install IDF to your computer as described at [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
- [Configure your environment](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables). This for example means `source ~/esp/esp-idf/export.sh`

Current versions of IDF promotes CMake/Python based build environment. It is used throughout wrapper `idf.py`. For example to build a project, you would use `idf.py build`. There is still legacy `make` base build toolchain. to make live easier, I've replaced the legacy `make` toolchain with `idf.py` based one through aliasing it the `Makefile`. E.g. `make build` will call `idf.py build` instead of legacy toolchain.
