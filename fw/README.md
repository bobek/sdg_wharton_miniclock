# Mini Clock Firmware

Firmware uses Espressif ESP-IDF. I've initially tried to get it working with PlatformIO, but was not successful as there are compatibility issues between cmake based IDF vs PlatformIO. Thus project is using IDF directly. I am on Linux (Debian), so the following steps are tested only there.

- Install IDF to your computer as described at [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
- [Configure your environment](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables). For example by `source ~/esp/esp-idf/export.sh`.
- Configure project with `idf.py menuconfig`. There is also a configuration section `Small Clock Configuration`.
- Build with `idf.py build` and flash with `idf.py flash`.

## Configuring local timezone

There is a `LOCAL_TZ_DATA` configuration option. It expects tzdata timezone configuration string. If on Linux, you can use the simple python script `get_tz_data.py` to get it from your local tzdata definition (typically located at `/usr/share/zoneinfo/`). For example:

```
$ ./get_tz_data.py Europe/Prague
CET-1CEST,M3.5.0,M10.5.0/3
$ ./get_tz_data.py America/New_York       
EST5EDT,M3.2.0,M11.1.0
```

You can get list of possible timezone names with `timedatectl list-timezones` or just simply check which files are in the zoneinfo directory.
