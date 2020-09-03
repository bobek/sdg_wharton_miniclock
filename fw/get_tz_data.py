#!/usr/bin/env python

import sys
import argparse

TZDATA_DIR = "/usr/share/zoneinfo/"


def get_tz_data_string(timezone):
    tz = open(TZDATA_DIR + timezone, "rb").read().split(b"\n")[-2]
    return tz.decode("utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Return tzdata string based on zones in " + TZDATA_DIR
    )
    parser.add_argument("timezone", type=str,
                        help="Timezone name to fetch tzdata definition for. "
                             "For example Europe/Prague. "
                             "You can use timedatectl list-timezones to get list of available zones. "
                             "Or just check files in the zone directory.")
    options = parser.parse_args()
    print(get_tz_data_string(options.timezone))
