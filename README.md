# Lian Li HydroShift II Bridge for SignalRGB

A SignalRGB add-on for the Lian Li HydroShift II LCD-S 360 AIO. It gives the block SignalRGB's
native LCD experience (picture and GIF picker, LCD faces, brightness) on the 480 x 480 panel and
exposes the 24-LED ring as a lit device, while a FanControl plugin keeps sole ownership of the
AIO's USB handle for pump control. SignalRGB renders; FanControl writes to the hardware.

Cooling is deliberately out of scope here. This plugin never sends a pump speed and never reads
the coolant sensor.

## Layout

```
network/
  LianLi_HydroShift2_Bridge/
    LianLi_HydroShift2_Bridge.js   the plugin (Type "network"), version 0.4.0
    LianLi_HydroShift2_Bridge.qml  its settings pane in the Add-ons page
    @SignalRGB/lcd                 link-time stand-in for SignalRGB's lcd module (see below)
```

SignalRGB scans the whole cloned repository, so the folder names are convention only. There is
no manifest file.

## How it works (0.4.0)

The plugin is a network-type device that never opens the AIO's USB handle. It imports
SignalRGB's lcd module, so the device gets the native LCD tab, grabs the 480 x 480 canvas as a
JPEG on every screen tick, cuts it into 2000-byte chunks behind an 18-byte header and sends them
over UDP to 127.0.0.1:48211, where the FanControl plugin that owns the USB handle reassembles the
frame and pushes it to the panel. The 24-LED ring rides the same socket as 72-byte colour
datagrams. Chunks are 2000 bytes because SignalRGB's udp module refuses larger datagrams
(measured: 2066 bytes sent, 4114 refused).

The stub at `@SignalRGB/lcd` exists because SignalRGB loads the same file in two JavaScript
engines. The device engine registers the real lcd module natively and the import binds to it. The
discovery engine has no lcd module, so the same import resolves to the stub next to the plugin,
the link succeeds and the discovery service runs. The stub marks itself with isShim and is never
used for frames.

Diagnostics: every log line and a five second counters summary are also sent as plain text to UDP
127.0.0.1:48213, where a local listener can record them; nothing listens there in normal use.

## Install

1. Open SignalRGB, click the Settings gear, go to the Add-ons tab.
2. Click Install An Add-on and paste:

   ```
   https://github.com/magnetgrouplabs/signalrgb-hydroshift2-bridge
   ```

3. Leave Branch on `main`, confirm the trust prompt, and restart SignalRGB when asked.

SignalRGB fetches and fast-forwards every installed add-on each time it starts, so updates pushed
to `main` arrive on the next full restart of SignalRGB. No click in the Add-ons tab is needed.

## Requirements

- SignalRGB 2.5.74 or later.
- The FanControl.HydroShift2 plugin running (it owns the USB device and listens on
  127.0.0.1:48211). Without it the device still appears in SignalRGB but nothing reaches the panel.

## Parameters

| Parameter | Default | Notes |
| --- | --- | --- |
| Bridge UDP Port | 48211 | Where the FanControl plugin listens |
| Stream Screen | on | |
| Screen Frames Per Second | 15 | 1 to 30; the ring is serviced at 30 Hz regardless |
| Screen Brightness | 100 | Sent to the receiver, which owns the panel's brightness command |
| Ring Lighting | on | Off sends one black frame and then nothing |
| Ring Brightness | 100 | Scaled on the host; the block has no ring brightness command |
| Reverse Ring Direction | off | Flips the index order, LED 1 stays in place |

## Status

- 0.2.1 (2026-09-01 17:00): device appeared, no LCD tab, no screen frames ever left SignalRGB.
- 0.3.0: static lcd import plus the shim file; the LCD tab appeared.
- 0.3.1 to 0.3.3: frame normalisation, 30 Hz render tick, diagnostics mirror, size probes.
- 0.4.0: 2000-byte chunks after the size probes showed the udp module refusing 4 KB and above.
  Requires the FanControl.HydroShift2 receiver build that accepts variable chunk lengths.

## Uninstall

Add-ons tab, select the add-on, Delete Add-on, then restart SignalRGB.

## License

MIT. See [LICENSE](LICENSE).
