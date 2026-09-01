# Lian Li HydroShift II LCD-S 360 for SignalRGB

A SignalRGB device plugin for the Lian Li HydroShift II LCD-S 360 AIO (USB `1CBE:A034`). It gives
the block SignalRGB's native LCD experience (picture and GIF picker, LCD faces, brightness) on the
480 x 480 panel, and exposes the 24-LED RGB ring around the block as a lit `Ring` subdevice so
effects and sync run on it like on any other device.

Cooling is deliberately out of scope. This plugin never sends a pump speed and never reads the
coolant sensor; those belong to a fan-control program. See "Coexisting with FanControl".

## Layout

```
usb/
  LianLi_HydroShift2_LCD.js    the plugin (Type "rawusb", VID 0x1CBE, PID 0xA034), version 1.0.0
```

SignalRGB scans the whole cloned repository and registers the plugin by the vendor and product id
it declares, so the folder name is convention only. There is no manifest file.

## Install

1. Open SignalRGB, click the Settings gear, go to the Add-ons tab.
2. Click Install An Add-on and paste:

   ```
   https://github.com/magnetgrouplabs/signalrgb-hydroshift2-bridge
   ```

3. Leave Branch on `main`, confirm the trust prompt, and restart SignalRGB when asked.

SignalRGB fetches and fast-forwards every installed add-on each time it starts, so updates pushed
to `main` arrive on the next full restart of SignalRGB.

## Requirements

- SignalRGB 2.5.74 or later.
- The AIO's internal USB cable connected (the USB path is the only control path).
- Nothing else holding the AIO's USB handle. The device is single-owner WinUSB: L-Connect 3 must
  not be running, and a FanControl plugin that owns this device must be disabled while this plugin
  is in use.

## Coexisting with FanControl

The block exposes one USB interface and Windows binds it to WinUSB, which allows one open handle
at a time. Until a sharing mechanism is in place, either SignalRGB (this plugin: LCD and ring) or
a fan-control plugin (pump curve) can hold the device, not both. With no host controlling the
pump, the block runs its own default of roughly 3100 RPM, which is its full speed and is safe.

All device I/O in this plugin goes through one transport object (`createUsbTransport`), so the
USB backend can be swapped for a shared-access driver later without touching the LCD or ring code.

## Parameters

| Group | Parameter | Default | Notes |
| --- | --- | --- | --- |
| screen | Brightness | 80 | Panel backlight, 0 to 100 |
| screen | Frames per second | 30 | 1 to 60 |
| screen | Rotation | 0 | 0, 90, 180, 270 |
| lighting | Ring Lighting | on | Off sends one black frame and then nothing |
| lighting | Ring Brightness | 100 | Scaled on the host; the block has no ring brightness command |
| lighting | Reverse Ring Direction | off | Flips the index order, LED 1 stays in place |
| lighting | Ring Offset | 0 | Rotates which physical LED is index 0 (0 to 23) |
| lighting | Ring Mode | Static | Static: one frame per colour change, rate limited by Ring Min Gap. Batch: multi-frame upload the block plays by itself |
| lighting | Ring Min Gap (ms) | 500 | Static mode rate limit; lower it if the block keeps up |
| lighting | Ring Refresh (s) | 0 | Static mode periodic re-push, 0 is off |
| lighting | Ring Batch Frames | 24 | Batch mode frames per upload |
| lighting | Ring Sample (ms) | 100 | Batch mode sampling and playback interval |
| advanced | Frame push mode | Single write | Chunked 1016 is a fallback for comparison |
| advanced | Read acknowledgements | on | Off makes every write fire-and-forget |

## Status

- LCD path: protocol verified on hardware (single bulk write, ack, panel holds the last frame).
- Ring: a solid colour push is verified on hardware. On 2026-09-01 a 77-push characterisation
  run (2 fps chase, 10 fps chase, one 24-frame self-playing upload) was acknowledged in full by
  the block; which of those visibly animated decides the right Ring Mode and Ring Min Gap.
  Which physical LED is index 0 and which way the indices run is not yet known; use Reverse and
  Offset.

## History

The network-type bridge plugin that shipped here as 0.2.1 to 0.4.0 (SignalRGB rendering, a
FanControl plugin owning the USB) is retired from this repository; its source and tests live in
the private HydroShift2 repository under `signalrgb/addon/`.

## Uninstall

Add-ons tab, select the add-on, Delete Add-on, then restart SignalRGB.

## License

MIT. See [LICENSE](LICENSE).
