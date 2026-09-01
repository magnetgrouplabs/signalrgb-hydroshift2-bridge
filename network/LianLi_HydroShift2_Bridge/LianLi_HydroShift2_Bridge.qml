// Settings pane for the HydroShift II bridge discovery service.
//
// SignalRGB looks for a .qml file sitting next to every network plugin's .js
// and named after it. When it is missing the engine logs
// "<name>.qml file doesn't exist! No interface available!" and
// "Failed to load UI." - the service then has no panel in the app, which is why
// nothing showed under Add-ons on the 2026-09-01 restart. Every shipped network
// plugin (WLED.qml, Cololight.qml, Twinkly.qml, Yeelight.qml, Govee.qml,
// Nanoleaf.qml, PhilipsHue.qml, leetdesk.qml, SRGBmods-WLC.qml) ships one.
//
// There is nothing to configure here: the endpoint is fixed at loopback and the
// bridge port lives on the device's own parameter list, not on the service. So
// this is a plain status card. `theme` and `discovery` are injected by the host.

Item {
	anchors.fill: parent

	Column {
		width: parent.width
		height: parent.height
		spacing: 10

		Rectangle {
			width: 450
			height: 120
			color: "#141414"
			radius: 5

			Column {
				x: 14
				y: 14
				width: parent.width - 28
				spacing: 8

				Text {
					color: theme.primarytextcolor
					text: "Lian Li HydroShift II Bridge"
					font.pixelSize: 16
					font.family: "Poppins"
					font.bold: true
				}

				Text {
					color: theme.primarytextcolor
					width: parent.width
					wrapMode: Text.WordWrap
					text: "Nothing is scanned. The plugin talks to the local HydroShift II bridge on 127.0.0.1 over UDP, so the device appears on its own as soon as the service starts."
					font.pixelSize: 13
					font.family: "Poppins"
				}

				Text {
					color: theme.primarytextcolor
					width: parent.width
					wrapMode: Text.WordWrap
					text: "The UDP port, frame rate, screen brightness and ring settings live on the device's own settings page, not here."
					font.pixelSize: 13
					font.family: "Poppins"
				}
			}
		}
	}
}
