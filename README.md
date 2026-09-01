# Lian Li HydroShift II Bridge - SignalRGB Add-on

A SignalRGB network plugin that lights the Lian Li HydroShift II AIO block
(24-LED RGB ring plus the LCD) by talking to a local bridge service over
loopback UDP. SignalRGB never touches the USB device directly; the bridge owns
the HID handle so it can coexist with fan-curve control.

## Layout

This repository follows the same shape as SignalRGB's own network add-ons
(for example `SRGBmods/plugins`, whose `network/WLED/WLED.js` + `WLED.qml` pair
is what the app loads):

```
network/
  LianLi_HydroShift2_Bridge/
    LianLi_HydroShift2_Bridge.js     the plugin
    LianLi_HydroShift2_Bridge.qml    its settings pane
```

SignalRGB scans the cloned repository, registers the plugin under the name its
`Name()` export returns, and loads the `.qml` sitting beside the `.js` as the
service's panel in the Add-ons page. There is no manifest file: SignalRGB add-on
repositories are plain git repositories, and the plugin's metadata lives in its
`Name()`, `Version()`, `Type()` and `Publisher()` exports.

## Install

1. Open SignalRGB, click the **Settings** gear, and go to the **Add-ons** tab.
2. Click **Install An Add-on**.
3. Paste this into the **Add-on Clone/Git Repository Url** field:

   ```
   https://github.com/magnetgrouplabs/signalrgb-hydroshift2-bridge
   ```

4. Leave **Branch** on `main`.
5. Confirm the trust prompt ("You should only install add-ons from trusted
   sources") by clicking **Install**.
6. SignalRGB will say changes to add-ons require a restart. Restart it.

Only `http`/`https` URLs are accepted. The installer validates the string and
rejects anything without an HTTP(S) scheme and a host, so a local folder path or
a `file://` URL will fail with "Invalid or unsafe repository url".

After the restart the plugin registers as **Lian Li HydroShift II Bridge** under
Add-ons. The bridge service must be running for a device to appear.

## Uninstall

Add-ons tab, select the add-on, **Delete Add-on**, then restart SignalRGB.

## License

MIT. See [LICENSE](LICENSE).
