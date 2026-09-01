import udp from "@SignalRGB/udp";

// @SignalRGB/lcd is deliberately NOT imported here. See lcdModule() below: a
// static import of it stops this plugin from ever producing a device.

// ---------------------------------------------------------------------------
// The block's RGB ring.
//
// 24 LEDs, one zone, per docs\ring-protocol.md (RING_LED_COUNT in the reference
// driver's h2_aio.rs, and the LCD-S shares the Circle's ring). They are laid out
// on the outer circle of a 13 x 13 canvas grid: `Ring 1` sits at 12 o'clock and
// the indices run clockwise, which puts every LED on its own cell and leaves the
// whole middle of the grid - a clear 7 x 7 - free, so the LCD image the device
// also carries is never sitting under a light.
//
// Whether the block's own index 0 is at 12 o'clock, and whether its indices run
// the same way round, is a bench question (ring-protocol.md, question 3). The
// `ringReverse` parameter flips the direction without touching this layout.
// ---------------------------------------------------------------------------

const RING_LED_COUNT = 24;
const RING_GRID = 13;
const RING_RADIUS = 6;

function buildRingPositions() {
	const centre = (RING_GRID - 1) / 2;
	const positions = [];

	for (let i = 0; i < RING_LED_COUNT; i++) {
		const angle = -Math.PI / 2 + (2 * Math.PI * i) / RING_LED_COUNT;

		positions.push([
			Math.round(centre + RING_RADIUS * Math.cos(angle)),
			Math.round(centre + RING_RADIUS * Math.sin(angle)),
		]);
	}

	return positions;
}

const RING_POSITIONS = buildRingPositions();
const RING_NAMES = RING_POSITIONS.map(function(_, i) { return "Ring " + (i + 1); });

export function Name() { return "Lian Li HydroShift II Bridge"; }
export function Version() { return "0.2.0"; }
export function Type() { return "network"; }
export function Publisher() { return "Magnet Group Labs"; }
export function Size() { return [RING_GRID, RING_GRID]; }
export function DefaultPosition() { return [240, 120]; }
export function DefaultScale() { return 1.0; }
export function DeviceType() { return "lcd"; }
export function LedNames() { return RING_NAMES.slice(); }
export function LedPositions() { return RING_POSITIONS.map(function(p) { return p.slice(); }); }
export function ConflictingProcesses() { return ["L-Connect-Service.exe", "L-Connect-Service-Watcher.exe"]; }

/* global
controller:readonly
bridgePort:readonly
targetFps:readonly
screenBrightness:readonly
ringEnabled:readonly
ringBrightness:readonly
ringReverse:readonly
*/
export function ControllableParameters() {
	return [
		{ property: "bridgePort", group: "settings", label: "Bridge UDP Port", type: "number", step: "1", min: "1024", max: "65535", default: "48211", live: false },
		{ property: "targetFps", group: "settings", label: "Frames Per Second", type: "number", step: "1", min: "1", max: "15", default: "5" },
		{ property: "screenBrightness", group: "settings", label: "Screen Brightness", type: "number", step: "5", min: "0", max: "100", default: "100" },
		{ property: "ringEnabled", group: "lighting", label: "Ring Lighting", type: "boolean", default: "true" },
		{ property: "ringBrightness", group: "lighting", label: "Ring Brightness", type: "number", step: "5", min: "0", max: "100", default: "100" },
		{ property: "ringReverse", group: "lighting", label: "Reverse Ring Direction", type: "boolean", default: "false" },
	];
}

// ---------------------------------------------------------------------------
// Wire contract. Mirrors docs\bridge-protocol.md; keep the two in step.
// ---------------------------------------------------------------------------

const MAGIC = [0x48, 0x53, 0x32, 0x42]; // "HS2B"
const PROTOCOL_VERSION = 1;
const HEADER_BYTES = 18;
const MAX_PAYLOAD_BYTES = 8192;
const MAX_FRAME_BYTES = 153600;
const MAX_DATAGRAM_BYTES = HEADER_BYTES + MAX_PAYLOAD_BYTES;

const FLAG_NO_FRAME = 0x01;   // bit0: heartbeat, carries no frame payload
const FLAG_BRIGHTNESS = 0x02; // bit1: payload is one screenBrightness byte
const FLAG_LIGHTING = 0x04;   // bit2: payload is N x RGB in ring order

const SCREEN_WIDTH = 480;
const SCREEN_HEIGHT = 480;
const HEARTBEAT_INTERVAL_MS = 1000;

// Ring cadence, per docs\bridge-protocol.md: at most 30 fps, only when the
// colours changed, and a re-send once a second so a receiver that started late
// converges. 34 ms rather than 33 keeps the cap at 30 datagrams in any second
// instead of 31.
const RING_PAYLOAD_BYTES = RING_LED_COUNT * 3; // 72
const RING_MIN_INTERVAL_MS = 34;
const RING_REFRESH_INTERVAL_MS = 1000;

// LCD.getFrame quality ladder. A 480x480 JPEG is normally well under the cap;
// the lower rungs only ever run if the encoder overshoots MAX_FRAME_BYTES.
const QUALITY_LADDER = [80, 60, 40];

const BRIDGE_HOST = "127.0.0.1";
const DEFAULT_PORT = 48211;
const CONTROLLER_ID = "hydroshift2-bridge-localhost";
const CONTROLLER_NAME = "HydroShift II Screen Bridge";

// Send address for the discovery service's inert socket. Never the bridge port:
// see the comment on DiscoveryService below.
const DISCOVERY_SINK_PORT = 48212;

// ---------------------------------------------------------------------------
// Datagram construction. Pure, no globals, so the test suite can drive it
// directly as well as through Initialize/Render/Shutdown.
// ---------------------------------------------------------------------------

function u16le(value) {
	return [value & 0xFF, (value >> 8) & 0xFF];
}

function u32le(value) {
	return [value & 0xFF, (value >>> 8) & 0xFF, (value >>> 16) & 0xFF, (value >>> 24) & 0xFF];
}

function buildHeader(flags, frameId, chunkIndex, chunkCount, chunkLen, totalLen) {
	return MAGIC
		.concat([PROTOCOL_VERSION & 0xFF, flags & 0xFF])
		.concat(u16le(frameId & 0xFFFF))
		.concat(u16le(chunkIndex & 0xFFFF))
		.concat(u16le(chunkCount & 0xFFFF))
		.concat(u16le(chunkLen & 0xFFFF))
		.concat(u32le(totalLen >>> 0));
}

function isJpeg(bytes) {
	return !!bytes
		&& bytes.length >= 4
		&& bytes[0] === 0xFF && bytes[1] === 0xD8
		&& bytes[bytes.length - 2] === 0xFF && bytes[bytes.length - 1] === 0xD9;
}

// Returns an array of datagrams, or null if the frame cannot be sent.
function buildFrameDatagrams(frameBytes, frameId) {
	if (!frameBytes || frameBytes.length === 0) { return null; }
	if (frameBytes.length > MAX_FRAME_BYTES) { return null; }
	if (!isJpeg(frameBytes)) { return null; }

	const totalLen = frameBytes.length;
	const chunkCount = Math.ceil(totalLen / MAX_PAYLOAD_BYTES);
	const datagrams = [];

	for (let index = 0; index < chunkCount; index++) {
		const start = index * MAX_PAYLOAD_BYTES;
		const chunk = Array.prototype.slice.call(frameBytes, start, start + MAX_PAYLOAD_BYTES);
		datagrams.push(buildHeader(0, frameId, index, chunkCount, chunk.length, totalLen).concat(chunk));
	}

	return datagrams;
}

// brightness === null gives the plain baseline heartbeat (18 bytes, no payload).
function buildHeartbeat(frameId, brightness) {
	if (brightness === null || brightness === undefined) {
		return buildHeader(FLAG_NO_FRAME, frameId, 0, 0, 0, 0);
	}

	const value = clamp(Math.round(brightness), 0, 100);

	return buildHeader(FLAG_NO_FRAME | FLAG_BRIGHTNESS, frameId, 0, 0, 1, 0).concat([value]);
}

// colours is a flat array of RING_LED_COUNT * 3 bytes, R, G, B per LED, in ring
// index order. Returns null rather than a malformed datagram: a lighting update
// that arrives wrong is worse than one that never arrives.
function buildLightingDatagram(colours, lightingFrameId) {
	if (!colours || colours.length !== RING_PAYLOAD_BYTES) { return null; }

	const payload = new Array(RING_PAYLOAD_BYTES);

	for (let i = 0; i < RING_PAYLOAD_BYTES; i++) {
		payload[i] = clamp(Math.round(Number(colours[i])), 0, 255);
	}

	return buildHeader(FLAG_LIGHTING, lightingFrameId, 0, 1, RING_PAYLOAD_BYTES, RING_PAYLOAD_BYTES).concat(payload);
}

function blackRing() {
	return new Array(RING_PAYLOAD_BYTES).fill(0);
}

function sameBytes(a, b) {
	if (!a || !b || a.length !== b.length) { return false; }

	for (let i = 0; i < a.length; i++) {
		if (a[i] !== b[i]) { return false; }
	}

	return true;
}

function clamp(value, low, high) {
	if (!isFinite(value)) { return low; }

	return value < low ? low : (value > high ? high : value);
}

function nextFrameId(current) {
	return (current + 1) & 0xFFFF;
}

// ---------------------------------------------------------------------------
// Device side
// ---------------------------------------------------------------------------

let socket = null;
let frameId = 0;
let lastFrameAt = 0;
let lastDatagramAt = 0;
let sentBrightness = -1;
let oversizeLogged = false;

// Lighting keeps its own frameId counter and its own clock; see
// docs\bridge-protocol.md, "Lighting and frame datagrams keep separate frameId
// counters and separate reassembly state".
let ringFrameId = 0;
let lastRingAt = 0;
let lastRingColours = null;
let ringBlanked = false;
let ringUnavailableLogged = false;

function log(message) {
	if (typeof device !== "undefined" && device && typeof device.log === "function") {
		device.log(message);
	} else if (typeof service !== "undefined" && service && typeof service.log === "function") {
		service.log(message);
	}
}

function currentPort() {
	const value = Math.round(Number(typeof bridgePort === "undefined" ? DEFAULT_PORT : bridgePort));

	return isFinite(value) && value >= 1024 && value <= 65535 ? value : DEFAULT_PORT;
}

function currentFps() {
	return clamp(Math.round(Number(typeof targetFps === "undefined" ? 5 : targetFps)), 1, 15);
}

function currentBrightness() {
	return clamp(Math.round(Number(typeof screenBrightness === "undefined" ? 100 : screenBrightness)), 0, 100);
}

function frameIntervalMs() {
	return Math.round(1000 / currentFps());
}

function ringIsEnabled() {
	return typeof ringEnabled === "undefined" ? true : (ringEnabled === true || ringEnabled === "true" || ringEnabled === 1);
}

function ringIsReversed() {
	return typeof ringReverse === "undefined" ? false : (ringReverse === true || ringReverse === "true" || ringReverse === 1);
}

function currentRingBrightness() {
	return clamp(Math.round(Number(typeof ringBrightness === "undefined" ? 100 : ringBrightness)), 0, 100);
}

function writeDatagram(datagram) {
	if (!socket) { return false; }
	if (datagram.length > MAX_DATAGRAM_BYTES) {
		log("Refused to send a " + datagram.length + " byte datagram; the cap is " + MAX_DATAGRAM_BYTES + ".");

		return false;
	}

	socket.write(datagram, BRIDGE_HOST, currentPort());

	return true;
}

// Frame and heartbeat datagrams stamp lastDatagramAt, which is what schedules
// the 1 s heartbeat. Lighting datagrams deliberately do not: bit2 traffic is not
// a sign of life to the receiver ("bit0 is what says the bridge is alive"), so
// letting it hold heartbeats off would silence the bridge from the receiver's
// point of view while the ring is streaming.
function sendDatagram(datagram, now) {
	if (writeDatagram(datagram)) { lastDatagramAt = now; }
}

function sendHeartbeat(now, withBrightness) {
	const brightness = withBrightness ? currentBrightness() : null;

	sendDatagram(buildHeartbeat(frameId, brightness), now);

	if (withBrightness) { sentBrightness = currentBrightness(); }
}

// Reads the 24 ring colours off the device canvas and flattens them to the 72
// wire bytes. Returns null when the host cannot hand us colours at all, which is
// the same class of unknown as @SignalRGB/lcd on a network plugin: better to
// send nothing than to stream black over whatever the block is showing.
function readRingColours() {
	if (typeof device === "undefined" || !device || typeof device.color !== "function") {
		if (!ringUnavailableLogged) {
			log("device.color is not available; the RGB ring stays off the wire.");
			ringUnavailableLogged = true;
		}

		return null;
	}

	const scale = currentRingBrightness() / 100;
	const reversed = ringIsReversed();
	const colours = new Array(RING_PAYLOAD_BYTES);

	for (let i = 0; i < RING_LED_COUNT; i++) {
		// Ring index i is fed from canvas LED (RING_LED_COUNT - i) % RING_LED_COUNT
		// when reversed, which keeps index 0 where it is and flips the direction
		// the colours travel. See the ring order caveat in signalrgb\README.md.
		const source = reversed ? (RING_LED_COUNT - i) % RING_LED_COUNT : i;
		const position = RING_POSITIONS[source];
		const colour = device.color(position[0], position[1]) || [0, 0, 0];

		colours[i * 3] = clamp(Math.round(colour[0] * scale), 0, 255);
		colours[i * 3 + 1] = clamp(Math.round(colour[1] * scale), 0, 255);
		colours[i * 3 + 2] = clamp(Math.round(colour[2] * scale), 0, 255);
	}

	return colours;
}

function sendRingColours(colours, now) {
	const datagram = buildLightingDatagram(colours, nextFrameId(ringFrameId));

	if (!datagram) {
		log("Dropped a lighting update: expected " + RING_PAYLOAD_BYTES + " colour bytes.");

		return false;
	}

	if (!writeDatagram(datagram)) { return false; }

	ringFrameId = nextFrameId(ringFrameId);
	lastRingColours = colours.slice();
	lastRingAt = now;

	return true;
}

function renderRing(now) {
	if (!ringIsEnabled()) {
		// One all-black datagram tells the receiver to blank the ring, then the
		// bridge stops talking about lighting entirely.
		if (ringBlanked) { return; }

		if (sendRingColours(blackRing(), now)) { ringBlanked = true; }

		return;
	}

	ringBlanked = false;

	if (now - lastRingAt < RING_MIN_INTERVAL_MS) { return; }

	const colours = readRingColours();

	if (!colours) { return; }

	const changed = !sameBytes(colours, lastRingColours);

	if (!changed && now - lastRingAt < RING_REFRESH_INTERVAL_MS) { return; }

	sendRingColours(colours, now);
}

// ---------------------------------------------------------------------------
// @SignalRGB/lcd, resolved lazily.
//
// This one file is evaluated in two different JavaScript engines. The device
// engine (Signal\Products\ThirdParty\Plugin\ThirdpartyJsPlugin.cpp) registers
// bus, hid, lcd, udp and the rest. The discovery engine
// (Signal\Discovery\DiscoveryService.cpp) registers a much smaller set -
// DeviceDiscovery ("service"), appInfo, permissions, tcp, udp, performance,
// base64 - and there is no lcd module in it.
//
// A static `import LCD from "@SignalRGB/lcd"` therefore fails to resolve when
// the discovery thread instantiates the module, the whole module errors out
// before DiscoveryService() can be read, and the plugin registers as a network
// service that never ticks, never announces a controller and never produces a
// device. That is exactly what happened on 2026-09-01: the log shows "Starting
// discovery thread for ...LianLi_HydroShift2_Bridge.js" and then nothing. Every
// shipped network plugin imports only modules the discovery engine also has
// (WLED and Yeelight import @SignalRGB/udp, Twinkly @SignalRGB/base64,
// Cololight/LeetDesk/SRGBmods-WLC nothing); @SignalRGB/lcd is imported only by
// USB plugins such as NZXT_Kraken_Elite.js.
//
// So the module is fetched at runtime, on the device side only, and every
// failure is swallowed: the ring, the heartbeat and the discovery service must
// all keep working on an engine that has no LCD at all.
let lcd = null;
let lcdRequested = false;
let lcdMissingLogged = false;

function lcdModule() {
	if (lcd) { return lcd; }

	// A host that hands the module over as a global needs nothing else. This is
	// also the seam the offline suite drives.
	if (typeof LCD !== "undefined" && LCD && typeof LCD.getFrame === "function") {
		lcd = LCD;

		return lcd;
	}

	if (!lcdRequested) {
		lcdRequested = true;

		try {
			// Not awaited: Initialize() and Render() both cope with a null lcd,
			// and the first frame simply waits for the promise to land.
			import("@SignalRGB/lcd").then(function(module) {
				lcd = (module && module.default) ? module.default : module;
				initializeLcd();
			}).catch(function(err) {
				log("Could not load @SignalRGB/lcd: " + err + ". The screen stays dark; the ring is unaffected.");
			});
		} catch (err) {
			log("Could not load @SignalRGB/lcd: " + err + ". The screen stays dark; the ring is unaffected.");
		}
	}

	return null;
}

function initializeLcd() {
	const module = lcdModule();

	if (!module) { return false; }

	module.initialize({ width: SCREEN_WIDTH, height: SCREEN_HEIGHT });

	return true;
}

function grabFrame() {
	const module = lcdModule();

	if (!module) {
		if (!lcdMissingLogged) {
			log("@SignalRGB/lcd is not available yet; no screen frames are being sent.");
			lcdMissingLogged = true;
		}

		return null;
	}

	lcdMissingLogged = false;

	for (let i = 0; i < QUALITY_LADDER.length; i++) {
		const frame = module.getFrame({ format: "JPEG", quality: QUALITY_LADDER[i] });

		if (!frame || frame.length === 0) { return null; }
		if (frame.length <= MAX_FRAME_BYTES) { return frame; }
	}

	return null;
}

export function Initialize() {
	if (typeof controller !== "undefined" && controller && controller.name) {
		device.setName(controller.name);
	}

	// The ring is announced through the same calls a top-level LED device uses.
	// A network device is created from discovery rather than from a static USB
	// descriptor, so the exported Size/LedNames/LedPositions are restated here;
	// both are guarded because neither call is documented as mandatory.
	if (typeof device.setSize === "function") { device.setSize([RING_GRID, RING_GRID]); }

	if (typeof device.setControllableLeds === "function") {
		device.setControllableLeds(RING_NAMES.slice(), RING_POSITIONS.map(function(p) { return p.slice(); }));
	}

	lcdMissingLogged = false;
	initializeLcd();

	socket = udp.createSocket();
	frameId = 0;
	lastFrameAt = 0;
	lastDatagramAt = 0;
	sentBrightness = -1;
	oversizeLogged = false;

	ringFrameId = 0;
	lastRingAt = 0;
	lastRingColours = null;
	ringBlanked = false;
	ringUnavailableLogged = false;

	device.setFrameRateTarget(currentFps());

	log("HydroShift II bridge sending to " + BRIDGE_HOST + ":" + currentPort() + " at " + currentFps() + " fps");

	// Announce ourselves straight away so the receiver stops its own renderer
	// before the first frame lands, and hand it the brightness at the same time.
	sendHeartbeat(Date.now(), true);
}

export function Render() {
	if (!socket) { return; }

	const now = Date.now();

	if (currentBrightness() !== sentBrightness) {
		sendHeartbeat(now, true);
	}

	// The ring runs on its own clock, up to 30 fps, so it must be serviced on
	// every tick rather than behind the screen's frame interval.
	renderRing(now);

	if (now - lastFrameAt < frameIntervalMs()) {
		if (now - lastDatagramAt >= HEARTBEAT_INTERVAL_MS) { sendHeartbeat(now, false); }

		return;
	}

	lastFrameAt = now;

	const frame = grabFrame();

	if (!frame) {
		if (!oversizeLogged) {
			log("Dropped a frame: LCD.getFrame did not return a JPEG within " + MAX_FRAME_BYTES + " bytes.");
			oversizeLogged = true;
		}

		if (now - lastDatagramAt >= HEARTBEAT_INTERVAL_MS) { sendHeartbeat(now, false); }

		return;
	}

	oversizeLogged = false;
	frameId = nextFrameId(frameId);

	const datagrams = buildFrameDatagrams(frame, frameId);

	if (!datagrams) {
		log("Dropped frame " + frameId + ": not a well formed JPEG.");

		return;
	}

	for (let i = 0; i < datagrams.length; i++) {
		sendDatagram(datagrams[i], now);
	}
}

export function Shutdown(suspend) {
	if (socket) {
		socket.close();
		socket = null;
	}

	log(suspend ? "HydroShift II bridge suspended." : "HydroShift II bridge stopped.");
}

export function onbridgePortChanged() {
	if (!socket) { return; }

	log("HydroShift II bridge now sending to " + BRIDGE_HOST + ":" + currentPort());
	sendHeartbeat(Date.now(), true);
}

export function ontargetFpsChanged() {
	if (!socket) { return; }

	device.setFrameRateTarget(currentFps());
	lastFrameAt = 0;
}

export function onscreenBrightnessChanged() {
	if (!socket) { return; }

	sendHeartbeat(Date.now(), true);
}

export function onringEnabledChanged() {
	if (!socket) { return; }

	// Turning it back on has to re-send even if the canvas has not moved, so the
	// change detector must forget what it last sent.
	lastRingColours = null;
	lastRingAt = 0;
	renderRing(Date.now());
}

export function onringBrightnessChanged() {
	if (!socket) { return; }

	lastRingColours = null;
	renderRing(Date.now());
}

export function onringReverseChanged() {
	if (!socket) { return; }

	lastRingColours = null;
	renderRing(Date.now());
}

export function ImageUrl() {
	return "https://assets.signalrgb.com/devices/brands/lian-li/coolers/galahad-ii-lcd.png";
}

// ---------------------------------------------------------------------------
// Discovery. There is nothing on the network to find: the receiver is the
// FanControl plugin on this machine. PluginCrawler refuses to load a
// Type() === "network" plugin that does not export DiscoveryService, so the
// service exists purely to announce one fixed loopback controller.
// ---------------------------------------------------------------------------

// A fabricated discovery reply. Every shipped network plugin builds its
// controller out of the `value` SignalRGB hands to DiscoveryService.Discovered()
// - Cololight.js, Govee.js and Nanoleaf.js all take {id, ip, port, response} -
// so the loopback endpoint is expressed in exactly that shape and pushed through
// exactly that path. Nothing here comes off the wire; the endpoint is fixed.
function loopbackDiscoveryValue() {
	return {
		id: CONTROLLER_ID,
		name: CONTROLLER_NAME,
		ip: BRIDGE_HOST,
		port: DEFAULT_PORT,
		response: { source: "loopback", host: BRIDGE_HOST, port: DEFAULT_PORT },
	};
}

class HydroShift2BridgeController {
	constructor(value) {
		this.initialized = false;
		this.updateWithValue(value, false);

		service.log("Constructed: " + this.name);
	}

	// Shipped controllers all expose updateWithValue(value) and notify the UI
	// through service.updateController(this); SignalRGB calls it when a device is
	// rediscovered, and the settings pane reads the properties back off it.
	updateWithValue(value, notify) {
		const source = value || loopbackDiscoveryValue();

		this.id = source.id;
		this.name = source.name || CONTROLLER_NAME;
		this.ip = source.ip;
		this.port = source.port;
		this.response = source.response;

		if (notify !== false) { service.updateController(this); }
	}

	// Called once per discovery tick off service.controllers, the way Govee.js
	// drives its controllers. announceController is what actually creates the
	// device, so it is guarded to fire exactly once.
	update() {
		if (this.initialized) { return; }

		this.initialized = true;
		service.updateController(this);
		service.announceController(this);
		service.log("Announced the local HydroShift II screen bridge.");
	}
}

export function DiscoveryService() {
	this.IconUrl = "https://assets.signalrgb.com/devices/brands/lian-li/coolers/galahad-ii-lcd.png";

	// SignalRGB's discovery thread reads exactly these properties off this object
	// - MDns, Hosts, UdpBroadcastAddress, UdpBroadcastPort, UdpListenPort - and
	// every shipped network plugin (WLED, Cololight, Twinkly, Yeelight, Govee,
	// Nanoleaf, PhilipsHue, LeetDesk, SRGBmods-WLC) sets one of them.
	//
	// Nothing is actually discovered here - the endpoint is fixed at loopback -
	// so the socket is deliberately inert: the send address points at a port
	// nothing listens on (NOT the bridge port, so the bridge never sees junk),
	// and UdpListenPort 0 lets the OS pick an ephemeral port so we can never
	// collide with the bridge's own bind. Yeelight uses UdpListenPort 0 the
	// same way. service.broadcast() is never called, so nothing is ever sent.
	this.UdpBroadcastAddress = BRIDGE_HOST;
	this.UdpBroadcastPort = DISCOVERY_SINK_PORT;
	this.UdpListenPort = 0;

	this.announcedLoopback = false;

	this.Initialize = function() {
		service.log("HydroShift II bridge service starting. Nothing is scanned; the endpoint is always " + BRIDGE_HOST + ".");
	};

	this.Update = function() {
		// First tick: hand ourselves the discovery reply the network would have
		// produced if there were anything out there to answer.
		if (!this.announcedLoopback) {
			this.announcedLoopback = true;
			this.Discovered(loopbackDiscoveryValue());
		}

		for (const cont of service.controllers) {
			if (cont.obj && typeof cont.obj.update === "function") { cont.obj.update(); }
		}
	};

	// The one path that creates a device, shaped exactly like Cololight.js:
	// getController -> addController -> updateController -> announceController on
	// a first sighting, updateWithValue on a repeat.
	this.Discovered = function(value) {
		if (!value || !value.id) { return; }

		const existing = service.getController(value.id);

		if (existing === undefined || existing === null) {
			const cont = new HydroShift2BridgeController(value);

			service.addController(cont);
			service.updateController(cont);
			service.announceController(cont);
			cont.initialized = true;
			service.log("Announced the local HydroShift II screen bridge.");
		} else {
			existing.updateWithValue(value);
		}
	};

	this.forceDiscover = function() {
		// Nothing to force: the endpoint is fixed at 127.0.0.1. Re-run the
		// loopback announce so a manual "add device" still lands.
		this.Discovered(loopbackDiscoveryValue());
	};
}
