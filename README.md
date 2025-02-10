![Photon Firmware](photon-firmware.png)

Photon is Open-Source firmware for pick and place feeders.

A feeder is a small machine that moves component tape very small, precise increments in order to automatically serve up new components to a pick and place machine.

Photon was originally developed as part of the LumenPnP project, but it is designed to support many types of hardware.

## Picking a Release

If you're looking to update to the latest stable release, use the release tagged `Latest`. Do NOT use a `Pre-Release` firmware, or one with `rc` in the name. We release lots of Release Candidate (denoted with an `rc` in the version name) firmware builds that are still being tested. If you're interested in testing the latest build, then the `rc` fimware is for you!

## Building and uploading

We use [platformio] for building, testing, and uploading this firmware. Refer to their docs for basic instructions on using platfomio.

The default `pio` environment is intended for use with a [Black Magic Probe][bmp] for uploading a debugging:

```sh
pio run
```

However, if you just want to program the feeder over UART, such as with the FTDI USB UART bridge included with the LumenPnP, you can use:

```sh
pio run -e photon-serial -t upload --upload-port /dev/tty...
```


[platformio]: http://platformio.org
[bmp]: https://black-magic.org/index.html
