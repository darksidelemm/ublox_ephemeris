# uBlox GPS Ephemeris Extractor
These scripts are intended to extract GPS Broadcast Ephemerides from a uBlox GPS, and output them in a [RINEX](ftp://igs.org/pub/data/format/rinex210.txt) Navigation Data compatible format.

This is primarily intended to provide an 'offline' alternative to the CDDIS-supplied broadcast ephemeris files, as required by the [RS](https://github.com/darksidelemm/RS) Vaisala RS92 Radiosonde decoder.

A lot of this repository is borrowed from tridge's [pyUblox](https://github.com/tridge/pyUblox) codebase.

Usage:
```
$ python ublox_ephemeris.py -p /dev/ttyUSB0 -b 115200 -o ephemeris.dat
```

NOTE: This is still a work-in-progress, and the RINEX output is currently un-usable due to missing fields.
