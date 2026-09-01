import LCD from "@SignalRGB/lcd";
import udp from "@SignalRGB/udp";

// LianLi_HydroShift2_LCD.js
//
// SignalRGB plugin for the Lian Li HydroShift II LCD-S 360 AIO (USB 1CBE:A034). It owns two
// things on the block: the 480x480 LCD panel and the 24-LED RGB ring. Both ride the same
// WinUSB bulk pair and the same DES-encrypted 512-byte command frames. The pump, the fan
// headers and the coolant sensor are NOT here and never will be: cooling belongs to
// FanControl, and no code path in this file can build opcode 0xFB (SyncPumpFan) or 0xFA
// (GetH2Params). lcd-plugin.test.js asserts that against every frame it captures.
//
// STATUS: version 1.0.0. The protocol layer below is verified byte-for-byte offline against
// the same known-answer vectors as the C# library (hs2-protocol.test.js), and every path in
// this file is exercised by lcd-plugin.test.js against a mock device. What that does and does
// not prove:
//
//   BENCH-PROVEN (hydroshift2-bench, ..\docs\ring-protocol.md "What the first hardware run
//   showed"):
//     - A single solid ring frame lands. One PushRgbData (0xFC), one 72-byte frame, trailer
//       interval 100, led count 24, literal-only tinyuz payload: the whole ring lit red.
//     - The ring channel order really is R, G, B.
//     - The firmware's tinyuz decoder accepts a literal-token-only stream.
//
//   DISPROVEN, and the reason the ring is not streamed here:
//     - 10 fps single-frame streaming is IGNORED by the block. Eighty pushes were acked and
//       the dot did not move. A PushRgbData is "install this animation and play it", not
//       "paint this now". Hence the Static mode below pushes only on change and never faster
//       than ringMinGapMs.
//
//   NOT TESTED ON HARDWARE:
//     - Ring "Batch" mode. The multi-frame upload exists in the wire format and the reference
//       says the firmware plays it onboard with no further host packets, but nothing in the
//       reference tree has ever sent one over the wire and no bench run has either. Treat it
//       as experimental; Static is the default for that reason.
//     - Where LED 0 sits on the physical ring and which way the indices run. ringOffset and
//       ringReverse exist so that can be corrected from the UI once someone has looked at the
//       block, without touching this file.
//     - Whether the ring holds its colour with no keepalive (ringRefreshS is the answer if it
//       does not).
//     - Whether interleaving PushRgbData with PushJpg on one handle upsets either.
//     - Whether the LCD panel itself accepts a push split across several bulk writes
//       ("Chunked 1016"); "Single write" is what the shipped Lian_Li_Universal_Screen_88.js
//       does on this same vendor and is the default.
//
// THE TRANSPORT SWAP POINT: createUsbTransport(device) below is the only place in this file
// that talks to the USB endpoints, and it is the only place that names the device transfer
// call at all (lcd-plugin.test.js greps the source to keep it that way). Everything else -
// sendCommand, pushJpeg, pushRing, the drain, the ack reads - takes a transport object with
// { name, write(bytes, timeoutMs), read(length, timeoutMs), drain() }. Swapping the block
// onto a different carrier (a UDP bridge to a host-side process, a named pipe to the
// FanControl plugin, a test double) means writing one more factory with that shape and
// choosing between them in Initialize. Nothing above the factory changes.
//
// Every device.* and LCD.* call used here is copied from plugins that ship with SignalRGB
// 2.5.74: Lian Li/Lian_Li_Universal_Screen_88.js (same vendor 0x1CBE, same DES frame format,
// same opcodes, the endpoint addresses and the reply-read loop) and Corsair/Corsair_XC7_LCD.js
// (the subdevice ring: createSubdevice / setSubdeviceLeds / setSubdeviceName /
// setSubdeviceImageUrl / setSubdeviceSize, device.subdeviceColor, SubdeviceController, and
// empty top-level LedNames/LedPositions). No API here is invented.
//
// SPDX-License-Identifier: MIT

// Inside an ES module on SignalRGB's Qt engine, globalThis, self, global and top-level this are
// all undefined, so the embedded module below would have nowhere to install itself and every
// HS2.* lookup would be a TypeError (the bare "Type error" SignalRGB logged for 1.0.0 to 1.0.2).
// It installs itself on this module-scoped object instead.
var HS2_ROOT = {};

// ==== BEGIN EMBEDDED hs2-protocol.js ====
// hs2-protocol.js
//
// Lian Li HydroShift II LCD-S 360 (USB 1CBE:A034) wire protocol, in dependency-free
// JavaScript. This is a straight port of src/HydroShift2.Protocol (Des.cs, Crc16Ccitt.cs,
// Frame.cs, Commands.cs, Opcode.cs, PanelInfo.cs, PumpMapping.cs, TimestampSource.cs),
// which is itself a port of PacketBuilder::build_winusb in lianli-devices/src/crypto.rs.
// The RGB ring layer is a port of TinyUz.cs, RingLayout.cs and LightingCommands.cs: the
// PushRgbData header, the literal-only tinyuz stream the payload is wrapped in, the frame
// layout and the whole header-plus-payload push stream.
//
// The requirement is byte-for-byte parity with the C# output. hs2-protocol.test.js checks
// every frame in tests/HydroShift2.Protocol.Tests/Vectors/vectors.json, so any drift here
// fails loudly rather than silently pushing a frame the block will reject.
//
// Deliberately ES5: `var`, plain functions, plain Arrays of byte numbers. It has to run in
// SignalRGB's plugin runtime (no require, no WebCrypto, no Node built-ins) and under plain
// Node for the tests, from the same source text. Arrays rather than Uint8Array because
// device.bulk_transfer wants a plain array.
//
// SPDX-License-Identifier: MIT

(function (root, factory) {
    "use strict";
    var api = factory();
    if (typeof module === "object" && module && module.exports) {
        module.exports = api;              // Node (the test harness)
    } else if (root) {
        root.HS2 = api;                    // SignalRGB / any plain-script runtime
    }
}(typeof HS2_ROOT !== "undefined" && HS2_ROOT ? HS2_ROOT   // a host module that declares one
    : typeof globalThis !== "undefined" ? globalThis
        : typeof self !== "undefined" ? self
            : typeof global !== "undefined" ? global
                : this, function () {
    "use strict";

    // ---------------------------------------------------------------------------------
    // DES, from scratch. Single DES, ECB core + CBC wrapper + PKCS#7.
    // ---------------------------------------------------------------------------------
    // No library and no WebCrypto in the plugin runtime, so the cipher is written out here.
    // FIPS 46-3 tables; the implementation is the textbook bit-array one rather than the
    // bitsliced/32-bit-half optimisation, because correctness is the whole point and the
    // load is trivial (one 512-byte frame = 63 blocks; at 5 fps that is ~315 blocks/sec).

    var PC1 = [
        57, 49, 41, 33, 25, 17, 9,
        1, 58, 50, 42, 34, 26, 18,
        10, 2, 59, 51, 43, 35, 27,
        19, 11, 3, 60, 52, 44, 36,
        63, 55, 47, 39, 31, 23, 15,
        7, 62, 54, 46, 38, 30, 22,
        14, 6, 61, 53, 45, 37, 29,
        21, 13, 5, 28, 20, 12, 4
    ];

    var PC2 = [
        14, 17, 11, 24, 1, 5,
        3, 28, 15, 6, 21, 10,
        23, 19, 12, 4, 26, 8,
        16, 7, 27, 20, 13, 2,
        41, 52, 31, 37, 47, 55,
        30, 40, 51, 45, 33, 48,
        44, 49, 39, 56, 34, 53,
        46, 42, 50, 36, 29, 32
    ];

    var SHIFTS = [1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1];

    var IP = [
        58, 50, 42, 34, 26, 18, 10, 2,
        60, 52, 44, 36, 28, 20, 12, 4,
        62, 54, 46, 38, 30, 22, 14, 6,
        64, 56, 48, 40, 32, 24, 16, 8,
        57, 49, 41, 33, 25, 17, 9, 1,
        59, 51, 43, 35, 27, 19, 11, 3,
        61, 53, 45, 37, 29, 21, 13, 5,
        63, 55, 47, 39, 31, 23, 15, 7
    ];

    var FP = [
        40, 8, 48, 16, 56, 24, 64, 32,
        39, 7, 47, 15, 55, 23, 63, 31,
        38, 6, 46, 14, 54, 22, 62, 30,
        37, 5, 45, 13, 53, 21, 61, 29,
        36, 4, 44, 12, 52, 20, 60, 28,
        35, 3, 43, 11, 51, 19, 59, 27,
        34, 2, 42, 10, 50, 18, 58, 26,
        33, 1, 41, 9, 49, 17, 57, 25
    ];

    var EXPANSION = [
        32, 1, 2, 3, 4, 5,
        4, 5, 6, 7, 8, 9,
        8, 9, 10, 11, 12, 13,
        12, 13, 14, 15, 16, 17,
        16, 17, 18, 19, 20, 21,
        20, 21, 22, 23, 24, 25,
        24, 25, 26, 27, 28, 29,
        28, 29, 30, 31, 32, 1
    ];

    var PBOX = [
        16, 7, 20, 21, 29, 12, 28, 17,
        1, 15, 23, 26, 5, 18, 31, 10,
        2, 8, 24, 14, 32, 27, 3, 9,
        19, 13, 30, 6, 22, 11, 4, 25
    ];

    var SBOX = [
        [14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7,
            0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8,
            4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0,
            15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13],
        [15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10,
            3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5,
            0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15,
            13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9],
        [10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8,
            13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1,
            13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7,
            1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12],
        [7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15,
            13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9,
            10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4,
            3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14],
        [2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9,
            14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6,
            4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14,
            11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3],
        [12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11,
            10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8,
            9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6,
            4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13],
        [4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1,
            13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6,
            1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2,
            6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12],
        [13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7,
            1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2,
            7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8,
            2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11]
    ];

    function bytesToBits(bytes, offset) {
        var bits = new Array(64), i, j, v;
        for (i = 0; i < 8; i++) {
            v = bytes[offset + i] & 0xFF;
            for (j = 0; j < 8; j++) bits[i * 8 + j] = (v >>> (7 - j)) & 1;
        }
        return bits;
    }

    function bitsToBytes(bits, out, offset) {
        var i, j, v;
        for (i = 0; i < 8; i++) {
            v = 0;
            for (j = 0; j < 8; j++) v = (v << 1) | bits[i * 8 + j];
            out[offset + i] = v & 0xFF;
        }
    }

    function permute(src, table) {
        var out = new Array(table.length), i;
        for (i = 0; i < table.length; i++) out[i] = src[table[i] - 1];
        return out;
    }

    function rotateLeft(bits, count) {
        return bits.slice(count).concat(bits.slice(0, count));
    }

    /** Expands an 8-byte key into the 16 48-bit round subkeys. */
    function desKeySchedule(keyBytes) {
        var pc1 = permute(bytesToBits(keyBytes, 0), PC1);
        var c = pc1.slice(0, 28);
        var d = pc1.slice(28, 56);
        var subkeys = [], r;
        for (r = 0; r < 16; r++) {
            c = rotateLeft(c, SHIFTS[r]);
            d = rotateLeft(d, SHIFTS[r]);
            subkeys.push(permute(c.concat(d), PC2));
        }
        return subkeys;
    }

    /** The Feistel f-function: 32-bit R plus a 48-bit subkey to 32 bits. */
    function feistel(r32, subkey) {
        var expanded = permute(r32, EXPANSION);
        var i, box, base, row, col, value;
        for (i = 0; i < 48; i++) expanded[i] ^= subkey[i];

        var sOut = new Array(32);
        for (box = 0; box < 8; box++) {
            base = box * 6;
            row = (expanded[base] << 1) | expanded[base + 5];
            col = (expanded[base + 1] << 3) | (expanded[base + 2] << 2)
                | (expanded[base + 3] << 1) | expanded[base + 4];
            value = SBOX[box][row * 16 + col];
            sOut[box * 4] = (value >>> 3) & 1;
            sOut[box * 4 + 1] = (value >>> 2) & 1;
            sOut[box * 4 + 2] = (value >>> 1) & 1;
            sOut[box * 4 + 3] = value & 1;
        }
        return permute(sOut, PBOX);
    }

    /** One 8-byte DES block, in place into `out` at `outOffset`. */
    function desBlock(input, inOffset, out, outOffset, subkeys, decrypt) {
        var bits = permute(bytesToBits(input, inOffset), IP);
        var l = bits.slice(0, 32);
        var r = bits.slice(32, 64);
        var round, f, next, i;

        for (round = 0; round < 16; round++) {
            f = feistel(r, subkeys[decrypt ? 15 - round : round]);
            next = new Array(32);
            for (i = 0; i < 32; i++) next[i] = l[i] ^ f[i];
            l = r;
            r = next;
        }
        // The halves are swapped once more before the final permutation.
        bitsToBytes(permute(r.concat(l), FP), out, outOffset);
    }

    function toByteArray(source) {
        var out = new Array(source.length), i;
        for (i = 0; i < source.length; i++) out[i] = source[i] & 0xFF;
        return out;
    }

    /** DES-CBC encrypt with PKCS#7 padding. Key and IV are 8 bytes each. */
    function desCbcEncryptPkcs7(plaintext, key, iv) {
        var subkeys = desKeySchedule(key);
        var padLength = 8 - (plaintext.length % 8);          // always 1..8, never 0
        var total = plaintext.length + padLength;
        var buf = new Array(total), out = new Array(total);
        var prev = toByteArray(iv).slice(0, 8);
        var block = new Array(8);
        var i, offset;

        for (i = 0; i < plaintext.length; i++) buf[i] = plaintext[i] & 0xFF;
        for (i = plaintext.length; i < total; i++) buf[i] = padLength;

        for (offset = 0; offset < total; offset += 8) {
            for (i = 0; i < 8; i++) block[i] = buf[offset + i] ^ prev[i];
            desBlock(block, 0, out, offset, subkeys, false);
            for (i = 0; i < 8; i++) prev[i] = out[offset + i];
        }
        return out;
    }

    /** DES-CBC decrypt, stripping PKCS#7. For tests and for reading captures. */
    function desCbcDecryptPkcs7(ciphertext, key, iv) {
        if (ciphertext.length === 0 || ciphertext.length % 8 !== 0) {
            throw new Error("ciphertext length must be a non-zero multiple of 8, got "
                + ciphertext.length);
        }
        var subkeys = desKeySchedule(key);
        var out = new Array(ciphertext.length);
        var prev = toByteArray(iv).slice(0, 8);
        var cipherBlock = new Array(8);
        var i, offset;

        for (offset = 0; offset < ciphertext.length; offset += 8) {
            for (i = 0; i < 8; i++) cipherBlock[i] = ciphertext[offset + i] & 0xFF;
            desBlock(cipherBlock, 0, out, offset, subkeys, true);
            for (i = 0; i < 8; i++) out[offset + i] ^= prev[i];
            for (i = 0; i < 8; i++) prev[i] = cipherBlock[i];
        }

        var padLength = out[out.length - 1];
        if (padLength < 1 || padLength > 8) throw new Error("bad PKCS#7 padding: " + padLength);
        for (i = out.length - padLength; i < out.length; i++) {
            if (out[i] !== padLength) throw new Error("bad PKCS#7 padding at " + i);
        }
        return out.slice(0, out.length - padLength);
    }

    // ---------------------------------------------------------------------------------
    // HydroShift II DES layer (Des.cs)
    // ---------------------------------------------------------------------------------

    /** ASCII "slv3tuzx". Both the DES key and the CBC IV. */
    var DES_KEY_AND_IV = [0x73, 0x6C, 0x76, 0x33, 0x74, 0x75, 0x7A, 0x78];

    var PLAINTEXT_LENGTH = 500;
    var CIPHERTEXT_LENGTH = 504;

    /** Encrypts the 500-byte command plaintext, producing exactly 504 bytes. */
    function encryptPlaintext(plaintext500) {
        if (!plaintext500 || plaintext500.length !== PLAINTEXT_LENGTH) {
            throw new Error("plaintext must be exactly " + PLAINTEXT_LENGTH + " bytes, got "
                + (plaintext500 ? plaintext500.length : "null"));
        }
        var cipher = desCbcEncryptPkcs7(plaintext500, DES_KEY_AND_IV, DES_KEY_AND_IV);
        if (cipher.length !== CIPHERTEXT_LENGTH) {
            throw new Error("expected " + CIPHERTEXT_LENGTH + " ciphertext bytes, got "
                + cipher.length);
        }
        return cipher;
    }

    /** Decrypts 504 ciphertext bytes back to the 500-byte plaintext. */
    function decryptCiphertext(ciphertext504) {
        if (!ciphertext504 || ciphertext504.length !== CIPHERTEXT_LENGTH) {
            throw new Error("ciphertext must be exactly " + CIPHERTEXT_LENGTH + " bytes, got "
                + (ciphertext504 ? ciphertext504.length : "null"));
        }
        return desCbcDecryptPkcs7(ciphertext504, DES_KEY_AND_IV, DES_KEY_AND_IV);
    }

    // ---------------------------------------------------------------------------------
    // CRC-16/CCITT (Crc16Ccitt.cs)
    // ---------------------------------------------------------------------------------
    // Polynomial 0x1021, init 0, no final XOR, no reflection. These are the CRC-16/XMODEM
    // parameters, so crc16ccitt("123456789") === 0x31C3.

    var CRC16_POLYNOMIAL = 0x1021;

    function crc16ccitt(data, offset, count) {
        if (!data) throw new Error("data is required");
        if (offset === undefined || offset === null) offset = 0;
        if (count === undefined || count === null) count = data.length - offset;
        if (offset < 0 || count < 0 || offset > data.length - count) {
            throw new Error("crc16ccitt range out of bounds: offset=" + offset
                + " count=" + count + " length=" + data.length);
        }

        var crc = 0, i, bit;
        for (i = offset; i < offset + count; i++) {
            crc = (crc ^ ((data[i] & 0xFF) << 8)) & 0xFFFF;
            for (bit = 0; bit < 8; bit++) {
                crc = (crc & 0x8000) !== 0
                    ? ((crc << 1) ^ CRC16_POLYNOMIAL) & 0xFFFF
                    : (crc << 1) & 0xFFFF;
            }
        }
        return crc;
    }

    // ---------------------------------------------------------------------------------
    // Frame (Frame.cs)
    // ---------------------------------------------------------------------------------
    // 500 bytes plaintext: [0] command, [1] 0x00, [2] 0x1A, [3] 0x6D,
    //                      [4..7] LE uint32 ms timestamp, [8..499] parameters (<=492).
    // DES-CBC + PKCS#7 turns that into 504. The 512-byte frame is those 504 bytes, then
    // six zeros, then [510] = 0xA1 and [511] = 0x1A.

    var FRAME_LENGTH = 512;
    var PARAMETERS_OFFSET = 8;
    var MAX_PARAMETER_LENGTH = PLAINTEXT_LENGTH - PARAMETERS_OFFSET;   // 492
    var MAGIC0 = 0x1A;
    var MAGIC1 = 0x6D;
    var TRAILER0 = 0xA1;
    var TRAILER1 = 0x1A;

    function buildFrame(command, parameters, timestampMs) {
        var paramLength = parameters ? parameters.length : 0;
        if (paramLength > MAX_PARAMETER_LENGTH) {
            throw new Error("parameters must be at most " + MAX_PARAMETER_LENGTH
                + " bytes, got " + paramLength);
        }
        var ts = timestampMs >>> 0;

        var plaintext = new Array(PLAINTEXT_LENGTH), i;
        for (i = 0; i < PLAINTEXT_LENGTH; i++) plaintext[i] = 0;
        plaintext[0] = command & 0xFF;
        plaintext[1] = 0x00;
        plaintext[2] = MAGIC0;
        plaintext[3] = MAGIC1;
        plaintext[4] = ts & 0xFF;
        plaintext[5] = (ts >>> 8) & 0xFF;
        plaintext[6] = (ts >>> 16) & 0xFF;
        plaintext[7] = (ts >>> 24) & 0xFF;
        for (i = 0; i < paramLength; i++) {
            plaintext[PARAMETERS_OFFSET + i] = parameters[i] & 0xFF;
        }

        var ciphertext = encryptPlaintext(plaintext);

        var frame = new Array(FRAME_LENGTH);
        for (i = 0; i < CIPHERTEXT_LENGTH; i++) frame[i] = ciphertext[i];
        for (i = CIPHERTEXT_LENGTH; i < 510; i++) frame[i] = 0;   // six unused bytes
        frame[510] = TRAILER0;
        frame[511] = TRAILER1;
        return frame;
    }

    function hasValidEnvelope(frame) {
        if (!frame || frame.length !== FRAME_LENGTH) return false;
        for (var i = CIPHERTEXT_LENGTH; i < 510; i++) if (frame[i] !== 0) return false;
        return frame[510] === TRAILER0 && frame[511] === TRAILER1;
    }

    /** Decrypts a 512-byte frame back to its 500-byte plaintext. Tests and captures only. */
    function decryptFramePlaintext(frame) {
        if (!frame || frame.length !== FRAME_LENGTH) {
            throw new Error("frame must be exactly " + FRAME_LENGTH + " bytes, got "
                + (frame ? frame.length : "null"));
        }
        return decryptCiphertext(frame.slice(0, CIPHERTEXT_LENGTH));
    }

    /** Splits a frame into command / timestamp / parameters. Tests and captures only. */
    function parseFrame(frame) {
        var p = decryptFramePlaintext(frame);
        return {
            command: p[0],
            reserved: p[1],
            magic0: p[2],
            magic1: p[3],
            timestampMs: (p[4] | (p[5] << 8) | (p[6] << 16) | (p[7] << 24)) >>> 0,
            parameters: p.slice(PARAMETERS_OFFSET, PARAMETERS_OFFSET + MAX_PARAMETER_LENGTH),
            isWellFormed: p[1] === 0x00 && p[2] === MAGIC0 && p[3] === MAGIC1
        };
    }

    function readUInt16BigEndian(buffer, offset) {
        return ((buffer[offset] << 8) | buffer[offset + 1]) & 0xFFFF;
    }

    function readUInt32BigEndian(buffer, offset) {
        return (((buffer[offset] << 24) >>> 0)
            + (buffer[offset + 1] << 16)
            + (buffer[offset + 2] << 8)
            + buffer[offset + 3]) >>> 0;
    }

    /**
     * Frame buffer level from a QueryBlock (0x7A) reply or a push ack: reply byte 8.
     * The reference waits for it to fall to 2 or less whenever an ack reports more than 3.
     */
    function readBufferLevel(reply) {
        if (!reply || reply.length < 9) {
            throw new Error("reply must be at least 9 bytes, got "
                + (reply ? reply.length : "null"));
        }
        return reply[8] & 0xFF;
    }

    // ---------------------------------------------------------------------------------
    // Opcodes (Opcode.cs)
    // ---------------------------------------------------------------------------------

    var Opcode = {
        GetVer: 0x0A,
        Reboot: 0x0B,
        Rotate: 0x0D,
        Brightness: 0x0E,
        FrameRate: 0x0F,
        GetH264Block: 0x11,
        WarnSwitch: 0x2E,
        SetClock: 0x33,
        StopClock: 0x34,
        PushJpg: 0x65,
        PushPng: 0x66,
        ClearPng: 0x67,
        HideShowFrames: 0x68,
        StartPlay: 0x79,
        QueryBlock: 0x7A,
        StopPlay: 0x7B,
        SwitchToDesktop: 0x96,
        SetWirelessThemeIndex: 0xF9,
        GetH2Params: 0xFA,
        SyncPumpFan: 0xFB,
        PushRgbData: 0xFC
    };

    // ---------------------------------------------------------------------------------
    // Panel parameters (PanelInfo.cs)
    // ---------------------------------------------------------------------------------

    var PanelInfo = {
        Width: 480,
        Height: 480,
        MaxFps: 60,
        JpegQuality: 85,
        MaxPayloadBytes: 153600,
        UsbVendorId: 0x1CBE,
        UsbProductId: 0xA034
    };

    // ---------------------------------------------------------------------------------
    // Timestamp source (TimestampSource.cs)
    // ---------------------------------------------------------------------------------
    // Bytes 4..7 of every frame. The firmware requires it to increase strictly, so the
    // source bumps by one whenever the clock has not moved: ts = raw > last ? raw : last + 1.
    //
    // CHANGED 2026-09-01: the origin is the Unix epoch, not this object's construction.
    // Until the AIO was rebound to libusbK.sys only one process could hold it, and an
    // origin of "whenever I started" was harmless. Now FanControl and SignalRGB write to
    // the same OUT endpoint, and two counters both starting near zero interleave
    // arbitrarily: whichever program launched second would hand the firmware timestamps
    // far below the other's. Date.now() taken modulo 2^32 gives both writers the same
    // origin, so their frames are ordered against each other by the wall clock. The C#
    // side does the same thing in MonotonicTimestampSource; see docs/usb-sharing.md.
    //
    // The value wraps every 2^32 ms (about 49.7 days). That is a property of the
    // protocol's 32 bit field, not of this choice of origin, and both writers wrap
    // together because both count from the same place.

    function MonotonicTimestampSource() {
        this._last = -1;
    }

    MonotonicTimestampSource.prototype.nextMs = function () {
        var raw = Date.now() >>> 0;
        var next = this._last < 0 ? raw
            : (raw > this._last ? raw : (this._last + 1) >>> 0);
        this._last = next >>> 0;
        return this._last;
    };

    /** Deterministic source for tests: a fixed value, or a scripted sequence. */
    function FixedTimestampSource(values) {
        this._values = Object.prototype.toString.call(values) === "[object Array]"
            ? values.slice() : [values];
        if (this._values.length === 0) throw new Error("at least one value is required");
        this._index = 0;
    }

    FixedTimestampSource.prototype.nextMs = function () {
        var value = this._values[this._index];
        if (this._index < this._values.length - 1) this._index++;
        return value >>> 0;
    };

    /**
     * Command builders accept either a timestamp source (anything with nextMs()) or a raw
     * millisecond number, so the known-answer tests can pass the vector's timestamp straight in.
     */
    function resolveTimestamp(timestamps) {
        if (timestamps === null || timestamps === undefined) {
            throw new Error("a timestamp source or a millisecond value is required");
        }
        if (typeof timestamps === "number") return timestamps >>> 0;
        if (typeof timestamps.nextMs === "function") return timestamps.nextMs() >>> 0;
        throw new Error("timestamps must be a number or expose nextMs()");
    }

    // ---------------------------------------------------------------------------------
    // Pump mapping (PumpMapping.cs)
    // ---------------------------------------------------------------------------------
    // The firmware value falls as RPM rises: 1590 at 1600 RPM down to 0 at 3200 RPM. The
    // reference does the arithmetic in f32, so every step goes through Math.fround; Rust's
    // f32::round() is half-away-from-zero, not banker's.

    var MIN_RPM = 1600;
    var MAX_RPM = 3200;

    var fr = Math.fround || function (x) { return x; };

    function roundHalfAwayFromZero(value) {
        return value >= 0 ? Math.floor(value + 0.5) : -Math.floor(-value + 0.5);
    }

    function pumpRpmToValue(rpm) {
        if (rpm < MIN_RPM) rpm = MIN_RPM;
        if (rpm > MAX_RPM) rpm = MAX_RPM;

        var r = fr(rpm);
        var value;
        if (r <= 1800) value = fr(1590 - fr(fr(r - 1600) * fr(0.95)));
        else if (r <= 2000) value = fr(1400 - fr(r - 1800));
        else if (r <= 2200) value = fr(1200 - fr(r - 2000));
        else if (r <= 2400) value = fr(1000 - fr(r - 2200));
        else if (r <= 2600) value = fr(800 - fr(r - 2400));
        else if (r <= 2800) value = fr(580 - fr(fr(r - 2600) * fr(1.11)));
        else if (r <= 3000) value = fr(330 - fr(fr(r - 2800) * fr(1.2)));
        else value = fr(90 - fr(fr(r - 3000) * fr(0.45)));

        var rounded = roundHalfAwayFromZero(value);
        if (rounded < 0) rounded = 0;
        if (rounded > 0xFFFF) rounded = 0xFFFF;
        return rounded;
    }

    function pumpPercentToRpm(percent) {
        if (typeof percent !== "number" || isNaN(percent)) {
            throw new Error("percent must be a number");
        }
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        return roundHalfAwayFromZero(MIN_RPM + percent / 100 * (MAX_RPM - MIN_RPM));
    }

    function pumpRpmToPercent(rpm) {
        if (rpm < MIN_RPM) rpm = MIN_RPM;
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        return (rpm - MIN_RPM) * 100 / (MAX_RPM - MIN_RPM);
    }

    // ---------------------------------------------------------------------------------
    // Command builders (Commands.cs)
    // ---------------------------------------------------------------------------------

    function payloadLengthParameters(length, name) {
        if (length <= 0 || length > PanelInfo.MaxPayloadBytes) {
            throw new Error(name + " must be between 1 and " + PanelInfo.MaxPayloadBytes
                + ", got " + length);
        }
        var v = length >>> 0;
        return [(v >>> 24) & 0xFF, (v >>> 16) & 0xFF, (v >>> 8) & 0xFF, v & 0xFF];
    }

    // PushRgbData (0xFC) parameter layout (LightingCommands.cs): an 8-byte parameter
    // block, first four bytes zero, then the payload length big-endian at offset 4 - not
    // offset 0 like PushJpg/PushPng above. Unlike payloadLengthParameters, the reference
    // only rejects a non-positive length; there is no upper bound.
    var PUSH_RGB_DATA_PARAMETER_LENGTH = 8;
    var PUSH_RGB_DATA_LENGTH_OFFSET = 4;

    function buildPushRgbDataParameters(payloadLength) {
        if (typeof payloadLength !== "number" || payloadLength <= 0) {
            throw new Error("payloadLength must be positive, got " + payloadLength);
        }
        var p = new Array(PUSH_RGB_DATA_PARAMETER_LENGTH), i;
        for (i = 0; i < PUSH_RGB_DATA_PARAMETER_LENGTH; i++) p[i] = 0;
        var v = payloadLength >>> 0;
        p[PUSH_RGB_DATA_LENGTH_OFFSET] = (v >>> 24) & 0xFF;
        p[PUSH_RGB_DATA_LENGTH_OFFSET + 1] = (v >>> 16) & 0xFF;
        p[PUSH_RGB_DATA_LENGTH_OFFSET + 2] = (v >>> 8) & 0xFF;
        p[PUSH_RGB_DATA_LENGTH_OFFSET + 3] = v & 0xFF;
        return p;
    }

    /**
     * The 492-byte SyncPumpFan parameter block: [0]=0xFF, [1]=0x0F, [2]=0xA2, [3]=0x00,
     * [4..5]=pump value big-endian, [6..8]=fan bytes, [12..13]=CRC-16/CCITT over 0..11
     * big-endian, the rest zero.
     */
    function buildSyncPumpFanParameters(pumpValue, fan1, fan2, fan3) {
        var p = new Array(MAX_PARAMETER_LENGTH), i;
        for (i = 0; i < MAX_PARAMETER_LENGTH; i++) p[i] = 0;
        p[0] = 0xFF;
        p[1] = 0x0F;
        p[2] = 0xA2;
        p[3] = 0x00;
        p[4] = (pumpValue >>> 8) & 0xFF;
        p[5] = pumpValue & 0xFF;
        p[6] = fan1 & 0xFF;
        p[7] = fan2 & 0xFF;
        p[8] = fan3 & 0xFF;
        var crc = crc16ccitt(p, 0, 12);
        p[12] = (crc >>> 8) & 0xFF;
        p[13] = crc & 0xFF;
        return p;
    }

    var commands = {
        /** Reads the firmware version string. No parameters. */
        getVer: function (timestamps) {
            return buildFrame(Opcode.GetVer, null, resolveTimestamp(timestamps));
        },

        /** Stops playback. No parameters. */
        stopPlay: function (timestamps) {
            return buildFrame(Opcode.StopPlay, null, resolveTimestamp(timestamps));
        },

        /** Stops the on-screen clock. One zero parameter byte, as the reference sends. */
        stopClock: function (timestamps) {
            return buildFrame(Opcode.StopClock, [0x00], resolveTimestamp(timestamps));
        },

        /** Sets the panel frame rate. Clamped to 1..60. */
        frameRate: function (timestamps, fps) {
            if (fps < 1) fps = 1;
            if (fps > PanelInfo.MaxFps) fps = PanelInfo.MaxFps;
            return buildFrame(Opcode.FrameRate, [fps & 0xFF], resolveTimestamp(timestamps));
        },

        /**
         * Sets the on-screen clock. Parameters are
         * [year hi, year lo, month, day, hour, minute, second, mode]. Local time; the
         * HydroShift II init sequence uses mode 2.
         */
        syncClock: function (timestamps, when, mode) {
            if (!(when instanceof Date)) throw new Error("when must be a Date");
            if (mode === undefined || mode === null) mode = 2;
            var year = when.getFullYear();
            if (year < 0 || year > 0xFFFF) throw new Error("year must fit in 16 bits");
            var parameters = [
                (year >>> 8) & 0xFF,
                year & 0xFF,
                (when.getMonth() + 1) & 0xFF,
                when.getDate() & 0xFF,
                when.getHours() & 0xFF,
                when.getMinutes() & 0xFF,
                when.getSeconds() & 0xFF,
                mode & 0xFF
            ];
            return buildFrame(Opcode.SetClock, parameters, resolveTimestamp(timestamps));
        },

        /** Header that precedes a JPEG push; the JPEG bytes follow it on the wire. */
        pushJpgHeader: function (timestamps, jpegLength) {
            return buildFrame(Opcode.PushJpg,
                payloadLengthParameters(jpegLength, "jpegLength"),
                resolveTimestamp(timestamps));
        },

        /** Header that precedes a PNG overlay push; the PNG bytes follow it on the wire. */
        pushPngHeader: function (timestamps, pngLength) {
            return buildFrame(Opcode.PushPng,
                payloadLengthParameters(pngLength, "pngLength"),
                resolveTimestamp(timestamps));
        },

        /** Clears the PNG overlay layer. No parameters. */
        clearPng: function (timestamps) {
            return buildFrame(Opcode.ClearPng, null, resolveTimestamp(timestamps));
        },

        /**
         * Header that precedes a PushRgbData ring-lighting push; the tinyuz-compressed
         * payload (colours plus the four trailer bytes) follows it on the wire. The one
         * lighting command the block has - see LightingCommands.cs.
         */
        pushRgbDataHeader: function (timestamps, payloadLength) {
            return buildFrame(Opcode.PushRgbData, buildPushRgbDataParameters(payloadLength),
                resolveTimestamp(timestamps));
        },

        /** Asks the device for its frame buffer level; read the answer with readBufferLevel. */
        queryBlock: function (timestamps) {
            return buildFrame(Opcode.QueryBlock, null, resolveTimestamp(timestamps));
        },

        /** Sets screen brightness. Clamped to 0..100. */
        brightness: function (timestamps, brightness) {
            if (brightness < 0) brightness = 0;
            if (brightness > 100) brightness = 100;
            return buildFrame(Opcode.Brightness, [brightness & 0xFF],
                resolveTimestamp(timestamps));
        },

        /** Sets screen rotation. Only the low two bits are sent; negatives become 0. */
        rotation: function (timestamps, rotation) {
            if (rotation < 0) rotation = 0;
            return buildFrame(Opcode.Rotate, [rotation & 0x03], resolveTimestamp(timestamps));
        },

        /** Requests AIO telemetry. No parameters; the reply is 512 plaintext bytes. */
        getH2Params: function (timestamps) {
            return buildFrame(Opcode.GetH2Params, null, resolveTimestamp(timestamps));
        },

        /** Sets pump speed (in RPM, mapped) and the three fan header bytes. */
        syncPumpFan: function (timestamps, pumpRpm, fan1, fan2, fan3) {
            return commands.syncPumpFanRaw(timestamps, pumpRpmToValue(pumpRpm),
                fan1, fan2, fan3);
        },

        /** As syncPumpFan, but with an already-mapped firmware value. */
        syncPumpFanRaw: function (timestamps, pumpValue, fan1, fan2, fan3) {
            return buildFrame(Opcode.SyncPumpFan,
                buildSyncPumpFanParameters(pumpValue, fan1, fan2, fan3),
                resolveTimestamp(timestamps));
        },

        /**
         * The wake preamble: StopPlay, StopClock, GetVer, in that order. After the LCD has
         * been in play mode the block ignores control commands until this re-arms the
         * channel. Send each frame with its own write and 512-byte read, ~150 ms apart.
         */
        wakePreamble: function (timestamps) {
            return [
                commands.stopPlay(timestamps),
                commands.stopClock(timestamps),
                commands.getVer(timestamps)
            ];
        }
    };

    // ---------------------------------------------------------------------------------
    // tinyuz (TinyUz.cs)
    // ---------------------------------------------------------------------------------
    // PushRgbData does not carry raw colour bytes: send_rgb_frames compresses the block
    // with tinyuz first. The vendored C++ compressor is a git submodule that is not
    // checked out in the on-disk reference, so the format was reconstructed from the ten
    // round-trip fixtures the reference did commit (lianli-devices/tests/fixtures/*.yuz).
    // decompress() below decodes all ten back to their inputs, which is what makes the
    // reconstruction checkable rather than a guess.
    //
    // Stream layout: four little-endian bytes of dictionary size, then a token stream.
    // Bits come out of control bytes least significant first, and those control bytes sit
    // in the same cursor the payload bytes come from.
    //   type bit 1: one literal, whose byte is the next byte from the cursor.
    //   type bit 0: a packed length (two-bit groups: payload bit then continuation bit);
    //     then, only when the previous token was a literal, a bit meaning "the same
    //     dictionary position as last time"; otherwise one dictionary-position byte. A
    //     non-zero position is a back reference of length + 2 bytes; a zero position makes
    //     the length a control code: 1 a literal run (three-bit groups plus 15, then that
    //     many verbatim bytes), 2 end of clip, 3 end of stream.
    //
    // compress() emits literal tokens only. That is the subset of the format that is
    // unconditionally present in the decoder, and it is why the declared dictionary size
    // is 1 - exactly what the reference compressor writes for its own match-free fixtures.

    var TINYUZ_DICTIONARY_SIZE_BYTES = 4;
    var TINYUZ_TYPE_BIT_COUNT = 8;
    var TINYUZ_MIN_DICTIONARY_MATCH_LENGTH = 2;
    var TINYUZ_MIN_LITERAL_RUN_LENGTH = 15;
    var TINYUZ_BIG_POSITION_FOR_LENGTH = (1 << 11) + (1 << 9) + (1 << 7) - 1;
    var TINYUZ_CONTROL_LITERAL_RUN = 1;
    var TINYUZ_CONTROL_CLIP_END = 2;
    var TINYUZ_CONTROL_STREAM_END = 3;

    /**
     * Bytes a literal-only stream costs for `length` input bytes: the four-byte head, the
     * data itself, one control bit per byte plus the six bits of the end token rounded up
     * to whole bytes, and the end token's dictionary-position byte.
     */
    function tinyUzCompressedLength(length) {
        if (typeof length !== "number" || length < 0 || Math.floor(length) !== length) {
            throw new Error("length must be a non-negative integer, got " + length);
        }
        var bits = length + (length > 0 ? 6 : 5);
        return TINYUZ_DICTIONARY_SIZE_BYTES + length + Math.floor((bits + 7) / 8) + 1;
    }

    /** Wraps `data` in a tinyuz stream built entirely from literal tokens. */
    function tinyUzCompress(data) {
        if (!data) throw new Error("data is required");

        var output = [];
        var state = { typesIndex: -1, typeCount: 0 };
        var i;

        // Dictionary size, little endian. No back reference is emitted, so a window of one
        // byte is enough - which is what the reference compressor writes for its own
        // match-free outputs.
        output.push(1);
        output.push(0);
        output.push(0);
        output.push(0);

        function emitBit(bit) {
            if (state.typeCount === 0) {
                state.typesIndex = output.length;
                output.push(0);
            }
            output[state.typesIndex] = (output[state.typesIndex] | (bit << state.typeCount)) & 0xFF;
            state.typeCount++;
            if (state.typeCount === TINYUZ_TYPE_BIT_COUNT) state.typeCount = 0;
        }

        for (i = 0; i < data.length; i++) {
            emitBit(1);
            output.push(data[i] & 0xFF);
        }

        // End of stream: type bit 0, packed length 3 (payload 0 / continue, then payload 1
        // / stop), the "not the same position" bit when anything has been written, and a
        // dictionary position of zero.
        emitBit(0);
        emitBit(0);
        emitBit(1);
        emitBit(1);
        emitBit(0);
        if (data.length > 0) emitBit(0);
        output.push(0);

        return output;
    }

    /**
     * Unwraps a tinyuz stream. Here so the compressor can be round-tripped and so the
     * reference's own fixtures can be decoded as known answers; the device never sends one.
     * `maxOutputLength` is an upper bound on the decoded size - decoding throws rather than
     * growing past it.
     */
    function tinyUzDecompress(stream, maxOutputLength) {
        if (!stream) throw new Error("stream is required");
        if (typeof maxOutputLength !== "number" || maxOutputLength < 0) {
            throw new Error("maxOutputLength must be non-negative, got " + maxOutputLength);
        }
        if (stream.length < TINYUZ_DICTIONARY_SIZE_BYTES) {
            throw new Error("a tinyuz stream is at least " + TINYUZ_DICTIONARY_SIZE_BYTES
                + " bytes, got " + stream.length);
        }

        var output = new Array(maxOutputLength), i;
        for (i = 0; i < maxOutputLength; i++) output[i] = 0;

        var cursor = {
            inPos: TINYUZ_DICTIONARY_SIZE_BYTES,
            outPos: 0,
            types: 0,
            typeCount: 0,
            haveDataBack: false,
            dictPosBack: 1
        };

        function nextByte() {
            if (cursor.inPos >= stream.length) throw new Error("tinyuz stream ended mid-token");
            return stream[cursor.inPos++] & 0xFF;
        }

        function readLowBits(bitCount) {
            var count = cursor.typeCount;
            var result = cursor.types;
            if (count >= bitCount) {
                cursor.typeCount = count - bitCount;
                cursor.types = result >> bitCount;
                return result & 0xFF;
            }
            var v = nextByte();
            bitCount -= count;
            cursor.typeCount = TINYUZ_TYPE_BIT_COUNT - bitCount;
            cursor.types = v >> bitCount;
            return (result | (v << count)) & 0xFF;
        }

        function unpackLength(readBit) {
            var payloadMask = (1 << (readBit - 1)) - 1;
            var continueMask = 1 << (readBit - 1);
            var value = 0, low;
            for (;;) {
                low = readLowBits(readBit);
                value = (value << (readBit - 1)) + (low & payloadMask);
                if ((low & continueMask) === 0) return value;
                value += 1;
            }
        }

        function append(value) {
            if (cursor.outPos >= output.length) {
                throw new Error("tinyuz stream decodes to more than the " + maxOutputLength
                    + " bytes allowed");
            }
            output[cursor.outPos++] = value & 0xFF;
        }

        var length, dictPos, raw, copy, run;
        for (;;) {
            if ((readLowBits(1) & 1) !== 0) {
                append(nextByte());
                cursor.haveDataBack = true;
                continue;
            }

            length = unpackLength(2);
            if (cursor.haveDataBack && (readLowBits(1) & 1) !== 0) {
                dictPos = cursor.dictPosBack;
            } else {
                raw = nextByte();
                if (raw >= 0x80) {
                    throw new Error("tinyuz dictionary positions of 128 or more use a "
                        + "multi-byte encoding that no reference fixture exercises, so this "
                        + "port refuses to guess at it");
                }
                dictPos = raw;
                length += dictPos > TINYUZ_BIG_POSITION_FOR_LENGTH ? 1 : 0;
            }

            if (dictPos !== 0) {
                copy = length + TINYUZ_MIN_DICTIONARY_MATCH_LENGTH;
                if (dictPos > cursor.outPos) {
                    throw new Error("tinyuz back reference of " + dictPos
                        + " reaches before the start of the output");
                }
                for (i = 0; i < copy; i++) append(output[cursor.outPos - dictPos]);
                cursor.dictPosBack = dictPos;

                // A back reference clears the flag: the "same position as last time" bit
                // only follows a literal. 04_palette_16 is the fixture that pins this -
                // read the flag as "anything has been written" and it decodes into nonsense
                // at its fifth token.
                cursor.haveDataBack = false;
                continue;
            }

            if (length === TINYUZ_CONTROL_LITERAL_RUN) {
                run = unpackLength(3) + TINYUZ_MIN_LITERAL_RUN_LENGTH;
                for (i = 0; i < run; i++) append(nextByte());
                cursor.dictPosBack = 1;
                cursor.haveDataBack = true;
                continue;
            }

            cursor.dictPosBack = 1;
            cursor.typeCount = 0;
            cursor.types = 0;

            if (length === TINYUZ_CONTROL_CLIP_END) continue;
            if (length === TINYUZ_CONTROL_STREAM_END) return output.slice(0, cursor.outPos);

            throw new Error("unknown tinyuz control code " + length);
        }
    }

    var TinyUz = {
        compress: tinyUzCompress,
        decompress: tinyUzDecompress,
        compressedLength: tinyUzCompressedLength,
        DICTIONARY_SIZE_BYTES: TINYUZ_DICTIONARY_SIZE_BYTES,
        TYPE_BIT_COUNT: TINYUZ_TYPE_BIT_COUNT,
        MIN_DICTIONARY_MATCH_LENGTH: TINYUZ_MIN_DICTIONARY_MATCH_LENGTH,
        MIN_LITERAL_RUN_LENGTH: TINYUZ_MIN_LITERAL_RUN_LENGTH,
        BIG_POSITION_FOR_LENGTH: TINYUZ_BIG_POSITION_FOR_LENGTH,
        CONTROL_LITERAL_RUN: TINYUZ_CONTROL_LITERAL_RUN,
        CONTROL_CLIP_END: TINYUZ_CONTROL_CLIP_END,
        CONTROL_STREAM_END: TINYUZ_CONTROL_STREAM_END
    };

    // ---------------------------------------------------------------------------------
    // Ring layout (RingLayout.cs)
    // ---------------------------------------------------------------------------------
    // 24 LEDs, three bytes each in red, green, blue order, one zone called "Ring". Both
    // HydroShift II variants have the same ring: is_square (PID 0xA034) only ever picks the
    // pump curve in h2_aio.rs and is never consulted for anything lighting related.

    var RING_LED_COUNT = 24;
    var RING_BYTES_PER_LED = 3;
    var RING_FRAME_LENGTH = RING_LED_COUNT * RING_BYTES_PER_LED;   // 72
    var RING_DEFAULT_INTERVAL_MS = 100;
    var RING_MAX_FRAME_COUNT = 65535;
    var RING_MAX_BRIGHTNESS_LEVEL = 4;
    var RING_BRIGHTNESS_OFF = 255;

    function ringOffsetOf(led) {
        if (typeof led !== "number" || led < 0 || led >= RING_LED_COUNT) {
            throw new Error("led must be between 0 and " + (RING_LED_COUNT - 1)
                + ", got " + led);
        }
        return led * RING_BYTES_PER_LED;
    }

    function requireRingFrame(frame) {
        if (!frame) throw new Error("frame is required");
        if (frame.length !== RING_FRAME_LENGTH) {
            throw new Error("a ring frame must be exactly " + RING_FRAME_LENGTH
                + " bytes, got " + frame.length);
        }
    }

    /** A frame with every LED off. */
    function ringOffFrame() {
        var frame = new Array(RING_FRAME_LENGTH), i;
        for (i = 0; i < RING_FRAME_LENGTH; i++) frame[i] = 0;
        return frame;
    }

    /** A frame with every LED set to one colour. */
    function ringSolidFrame(red, green, blue) {
        var frame = new Array(RING_FRAME_LENGTH), led, at;
        for (led = 0; led < RING_LED_COUNT; led++) {
            at = led * RING_BYTES_PER_LED;
            frame[at] = red & 0xFF;
            frame[at + 1] = green & 0xFF;
            frame[at + 2] = blue & 0xFF;
        }
        return frame;
    }

    /** Writes one LED's colour into an existing frame. */
    function ringSetLed(frame, led, red, green, blue) {
        requireRingFrame(frame);
        var at = ringOffsetOf(led);
        frame[at] = red & 0xFF;
        frame[at + 1] = green & 0xFF;
        frame[at + 2] = blue & 0xFF;
        return frame;
    }

    /** A frame with a single LED lit and the rest off. */
    function ringDotFrame(led, red, green, blue) {
        return ringSetLed(ringOffFrame(), led, red, green, blue);
    }

    /** The reference's own brightness scale: 0..4, with 255 meaning off. */
    function ringScaleLevel(level) {
        if (level === RING_BRIGHTNESS_OFF) return 0;
        return level > RING_MAX_BRIGHTNESS_LEVEL ? RING_MAX_BRIGHTNESS_LEVEL : level;
    }

    function ringScaleBy(frame, scale) {
        var scaled = new Array(frame.length), i, value;
        for (i = 0; i < frame.length; i++) {
            value = roundHalfAwayFromZero((frame[i] & 0xFF) * scale);
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            scaled[i] = value;
        }
        return scaled;
    }

    /** Scales a frame by one of the reference's five brightness levels. */
    function ringScale(frame, level) {
        requireRingFrame(frame);
        return ringScaleBy(frame, fr(ringScaleLevel(level) / 4));
    }

    /**
     * Scales a frame by a 0..100 percentage, half away from zero and clamped, so the ring
     * can be dimmed on the same axis the panel's backlight uses. There is no ring-brightness
     * opcode on the wire; every brightness change is the host rescaling and pushing again.
     */
    function ringScalePercent(frame, percent) {
        requireRingFrame(frame);
        if (typeof percent !== "number" || isNaN(percent)) {
            throw new Error("percent must be a number");
        }
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        return ringScaleBy(frame, fr(percent / 100));
    }

    var RingLayout = {
        LedCount: RING_LED_COUNT,
        BytesPerLed: RING_BYTES_PER_LED,
        FrameLength: RING_FRAME_LENGTH,
        RedOffset: 0,
        GreenOffset: 1,
        BlueOffset: 2,
        ZoneIndex: 0,
        ZoneName: "Ring",
        CircleProductId: 0xA021,
        SquareProductId: 0xA034,
        DefaultIntervalMs: RING_DEFAULT_INTERVAL_MS,
        MaxFrameCount: RING_MAX_FRAME_COUNT,
        MaxBrightnessLevel: RING_MAX_BRIGHTNESS_LEVEL,
        BrightnessOff: RING_BRIGHTNESS_OFF,
        ledCountFor: function (productId) {
            return productId === 0xA021 || productId === 0xA034 ? RING_LED_COUNT : 0;
        },
        offsetOf: ringOffsetOf,
        offFrame: ringOffFrame,
        solidFrame: ringSolidFrame,
        dotFrame: ringDotFrame,
        setLed: ringSetLed,
        scaleLevel: ringScaleLevel,
        scale: ringScale,
        scalePercent: ringScalePercent
    };

    // ---------------------------------------------------------------------------------
    // Ring lighting commands (LightingCommands.cs)
    // ---------------------------------------------------------------------------------
    // The wired path has exactly one lighting command, PushRgbData (0xFC), and it always
    // carries per-LED colour: no effect-mode opcode, no ring-brightness opcode. The stream
    // is the 512-byte header followed immediately by
    //   [ tinyuz(colour block) ][ frames hi ][ frames lo ][ interval ms ][ led count ]
    // in the same bulk write.

    var RING_TRAILER_LENGTH = 4;
    var RING_MIN_INTERVAL_MS = 1;
    var RING_MAX_INTERVAL_MS = 255;

    /** True for a bare frame (an array of colour bytes) rather than a list of frames. */
    function isSingleRingFrame(frames) {
        return frames.length > 0 && typeof frames[0] === "number";
    }

    function ringFrameList(frames) {
        if (!frames) throw new Error("frames is required");
        return isSingleRingFrame(frames) ? [frames] : frames;
    }

    /**
     * Flattens ring frames into the uncompressed block. A frame shorter than 72 bytes is
     * padded with black, matching frame.get(led).copied().unwrap_or([0, 0, 0]).
     */
    function ringBuildBlock(frames) {
        var list = ringFrameList(frames);
        if (list.length === 0) throw new Error("at least one frame is required");
        if (list.length > RING_MAX_FRAME_COUNT) {
            throw new Error("at most " + RING_MAX_FRAME_COUNT
                + " frames fit in one payload, got " + list.length);
        }

        var block = new Array(list.length * RING_FRAME_LENGTH), i, j, frame, at;
        for (i = 0; i < block.length; i++) block[i] = 0;

        for (i = 0; i < list.length; i++) {
            frame = list[i];
            if (!frame) throw new Error("frame " + i + " is null");
            if (frame.length > RING_FRAME_LENGTH) {
                throw new Error("frame " + i + " is " + frame.length + " bytes, longer than the "
                    + RING_FRAME_LENGTH + " a ring frame holds");
            }
            at = i * RING_FRAME_LENGTH;
            for (j = 0; j < frame.length; j++) block[at + j] = frame[j] & 0xFF;
        }
        return block;
    }

    /** The payload that follows the header: the compressed block then the four trailer bytes. */
    function ringBuildPayload(compressedBlock, frameCount, intervalMs, ledCount) {
        if (!compressedBlock) throw new Error("compressedBlock is required");
        if (typeof frameCount !== "number" || frameCount < 1 || frameCount > RING_MAX_FRAME_COUNT) {
            throw new Error("frame count must be between 1 and " + RING_MAX_FRAME_COUNT
                + ", got " + frameCount);
        }
        if (typeof intervalMs !== "number"
            || intervalMs < RING_MIN_INTERVAL_MS || intervalMs > RING_MAX_INTERVAL_MS) {
            throw new Error("interval must be between " + RING_MIN_INTERVAL_MS + " and "
                + RING_MAX_INTERVAL_MS + "; the trailer carries it in one byte. Got "
                + intervalMs);
        }
        if (typeof ledCount !== "number" || ledCount < 0 || ledCount > 255) {
            throw new Error("led count must be between 0 and 255, got " + ledCount);
        }

        var payload = new Array(compressedBlock.length + RING_TRAILER_LENGTH), i;
        for (i = 0; i < compressedBlock.length; i++) payload[i] = compressedBlock[i] & 0xFF;
        payload[compressedBlock.length] = (frameCount >>> 8) & 0xFF;
        payload[compressedBlock.length + 1] = frameCount & 0xFF;
        payload[compressedBlock.length + 2] = intervalMs & 0xFF;
        payload[compressedBlock.length + 3] = ledCount & 0xFF;
        return payload;
    }

    /**
     * The whole PushRgbData command as one linear stream: the 512-byte header then its
     * payload, ready for a single bulk write. `frames` is either one frame (an array of
     * colour bytes) or an array of frames; more than one makes the firmware loop them by
     * itself at `intervalMs`, and the reference only ever sends one.
     */
    function ringPushStream(timestamps, frames, intervalMs) {
        if (intervalMs === undefined || intervalMs === null) {
            intervalMs = RING_DEFAULT_INTERVAL_MS;
        }
        var list = ringFrameList(frames);
        var block = ringBuildBlock(list);
        var payload = ringBuildPayload(tinyUzCompress(block), list.length, intervalMs,
            RING_LED_COUNT);
        return concatBytes(commands.pushRgbDataHeader(timestamps, payload.length), payload);
    }

    var ring = {
        buildBlock: ringBuildBlock,
        buildPayload: ringBuildPayload,
        pushStream: ringPushStream,

        /** Sets every LED on the ring to one colour. */
        solid: function (timestamps, red, green, blue, intervalMs) {
            return ringPushStream(timestamps, [ringSolidFrame(red, green, blue)], intervalMs);
        },

        /** Turns every LED on the ring off. */
        off: function (timestamps, intervalMs) {
            return ringPushStream(timestamps, [ringOffFrame()], intervalMs);
        },

        TRAILER_LENGTH: RING_TRAILER_LENGTH,
        MIN_INTERVAL_MS: RING_MIN_INTERVAL_MS,
        MAX_INTERVAL_MS: RING_MAX_INTERVAL_MS
    };

    // ---------------------------------------------------------------------------------
    // Bulk transport helpers
    // ---------------------------------------------------------------------------------
    // SignalRGB caps a bulk write at 1025 bytes, so a 512-byte header plus up to 153,600
    // JPEG bytes cannot go out in one call the way the Linux driver sends it. These build
    // the stream and cut it up; whether the firmware tolerates the split is the open bench
    // question (see README).

    var DEFAULT_CHUNK_SIZE = 1016;   // what Corsair_Elite_Capellix_LCD.js uses

    /** Concatenates the PushJpg header frame and the JPEG bytes into one linear stream. */
    function buildJpgPushStream(timestamps, jpegBytes) {
        var header = commands.pushJpgHeader(timestamps, jpegBytes.length);
        return concatBytes(header, jpegBytes);
    }

    /** Concatenates the PushPng header frame and the PNG bytes into one linear stream. */
    function buildPngPushStream(timestamps, pngBytes) {
        var header = commands.pushPngHeader(timestamps, pngBytes.length);
        return concatBytes(header, pngBytes);
    }

    function concatBytes(a, b) {
        var out = new Array(a.length + b.length), i;
        for (i = 0; i < a.length; i++) out[i] = a[i] & 0xFF;
        for (i = 0; i < b.length; i++) out[a.length + i] = b[i] & 0xFF;
        return out;
    }

    /** Splits a stream into pieces of at most `maxChunk` bytes, in order. */
    function chunkStream(stream, maxChunk) {
        if (!maxChunk || maxChunk < 1) maxChunk = DEFAULT_CHUNK_SIZE;
        var chunks = [], offset;
        for (offset = 0; offset < stream.length; offset += maxChunk) {
            chunks.push(stream.slice(offset, Math.min(offset + maxChunk, stream.length)));
        }
        return chunks;
    }

    /** Puts the pieces back together. Used by the tests to prove the split is lossless. */
    function reassembleChunks(chunks) {
        var out = [], i, j;
        for (i = 0; i < chunks.length; i++) {
            for (j = 0; j < chunks[i].length; j++) out.push(chunks[i][j] & 0xFF);
        }
        return out;
    }

    // ---------------------------------------------------------------------------------
    // Small utilities
    // ---------------------------------------------------------------------------------

    function bytesToHex(bytes) {
        var hex = "", i, b;
        for (i = 0; i < bytes.length; i++) {
            b = (bytes[i] & 0xFF).toString(16);
            hex += b.length === 1 ? "0" + b : b;
        }
        return hex;
    }

    function hexToBytes(hex) {
        var clean = String(hex).replace(/[^0-9a-fA-F]/g, "");
        if (clean.length % 2 !== 0) throw new Error("hex string must have an even length");
        var out = new Array(clean.length / 2), i;
        for (i = 0; i < out.length; i++) {
            out[i] = parseInt(clean.substr(i * 2, 2), 16);
        }
        return out;
    }

    function asciiToBytes(text) {
        var out = new Array(text.length), i;
        for (i = 0; i < text.length; i++) out[i] = text.charCodeAt(i) & 0xFF;
        return out;
    }

    // ---------------------------------------------------------------------------------

    return {
        // DES
        DES_KEY_AND_IV: DES_KEY_AND_IV,
        desKeySchedule: desKeySchedule,
        desCbcEncryptPkcs7: desCbcEncryptPkcs7,
        desCbcDecryptPkcs7: desCbcDecryptPkcs7,
        encryptPlaintext: encryptPlaintext,
        decryptCiphertext: decryptCiphertext,
        PLAINTEXT_LENGTH: PLAINTEXT_LENGTH,
        CIPHERTEXT_LENGTH: CIPHERTEXT_LENGTH,

        // CRC
        crc16ccitt: crc16ccitt,
        CRC16_POLYNOMIAL: CRC16_POLYNOMIAL,

        // Frame
        buildFrame: buildFrame,
        parseFrame: parseFrame,
        decryptFramePlaintext: decryptFramePlaintext,
        hasValidEnvelope: hasValidEnvelope,
        readUInt16BigEndian: readUInt16BigEndian,
        readUInt32BigEndian: readUInt32BigEndian,
        readBufferLevel: readBufferLevel,
        FRAME_LENGTH: FRAME_LENGTH,
        PARAMETERS_OFFSET: PARAMETERS_OFFSET,
        MAX_PARAMETER_LENGTH: MAX_PARAMETER_LENGTH,

        // Opcodes and panel
        Opcode: Opcode,
        PanelInfo: PanelInfo,

        // Timestamps
        MonotonicTimestampSource: MonotonicTimestampSource,
        FixedTimestampSource: FixedTimestampSource,

        // Pump
        pumpRpmToValue: pumpRpmToValue,
        pumpPercentToRpm: pumpPercentToRpm,
        pumpRpmToPercent: pumpRpmToPercent,
        PUMP_MIN_RPM: MIN_RPM,
        PUMP_MAX_RPM: MAX_RPM,
        buildSyncPumpFanParameters: buildSyncPumpFanParameters,
        buildPushRgbDataParameters: buildPushRgbDataParameters,

        // Commands
        commands: commands,

        // Ring lighting
        TinyUz: TinyUz,
        RingLayout: RingLayout,
        ring: ring,

        // Transport
        DEFAULT_CHUNK_SIZE: DEFAULT_CHUNK_SIZE,
        buildJpgPushStream: buildJpgPushStream,
        buildPngPushStream: buildPngPushStream,
        chunkStream: chunkStream,
        reassembleChunks: reassembleChunks,
        concatBytes: concatBytes,

        // Utilities
        bytesToHex: bytesToHex,
        hexToBytes: hexToBytes,
        asciiToBytes: asciiToBytes
    };
}));
// ==== END EMBEDDED hs2-protocol.js ====
/**
 * The protocol module the block above installed on the global object. SignalRGB plugins are
 * ES modules with no loader for local files - `import` only resolves SignalRGB's own
 * packages such as "@SignalRGB/lcd" - so hs2-protocol.js is embedded verbatim rather than
 * imported. hs2-protocol.test.js fails if the copy above drifts from the tested original.
 */
const HS2 = HS2_ROOT.HS2
    || (typeof globalThis !== "undefined" && globalThis.HS2)
    || (typeof self !== "undefined" && self.HS2)
    || (typeof global !== "undefined" && global.HS2);

// ---------------------------------------------------------------------------------------
// Device identity
// ---------------------------------------------------------------------------------------

export function Name() { return "Lian Li HydroShift II LCD-S 360"; }
export function Version() { return "1.0.6"; }
export function VendorId() { return 0x1CBE; }
export function ProductId() { return 0xA034; }
export function Publisher() { return "Magnet Group Labs"; }
export function Size() { return [1, 1]; }
export function DefaultPosition() { return [240, 120]; }
export function DefaultScale() { return 1.0; }
export function DeviceType() { return "lcd"; }

/**
 * "rawusb" rather than "HID": this device speaks WinUSB bulk transfers, not HID reports.
 * Confirmed from Lian_Li_Universal_Screen_88.js, which returns the lowercase string. The
 * developer docs write it "RAWUSB"; SignalRGB compares case-insensitively in every shipped
 * plugin observed, and the lowercase form is what the sibling 0x1CBE device uses, so this
 * matches the one plugin known to bind successfully to this vendor.
 */
export function Type() { return "rawusb"; }

/**
 * The RGB ring is a subdevice, so the plugin has to declare itself a subdevice controller.
 * Corsair_XC7_LCD.js:21-23 does exactly this for the same arrangement - an LCD with a ring
 * around it - and it is the only shipped plugin of that exact shape.
 */
export function SubdeviceController() { return true; }

/**
 * The ring lives in a "Ring" subdevice and the top level stays empty, so nothing sits on top
 * of the LCD image. Corsair_XC7_LCD.js:131-137 returns the same pair of empty arrays while
 * :236-245 builds the subdevice.
 */
export function LedNames() { return []; }
export function LedPositions() { return []; }

/**
 * L-Connect grabs the same handle and fights over it. Copied from
 * Lian_Li_Universal_Screen_88.js. L-Connect 3 is already uninstalled on this machine, so
 * this is belt and braces.
 *
 * FanControl.exe is deliberately NOT here. The FanControl plugin holds this same device for
 * pump and fan control and the two are expected to coexist; declaring it a conflict would
 * ask SignalRGB to shut cooling down, which is exactly backwards.
 */
export function ConflictingProcesses() {
    return ["L-Connect-Service.exe", "L-Connect-Service-Watcher.exe"];
}

export function ImageUrl() {
    return "https://assets.signalrgb.com/devices/brands/lian-li/aio/hydroshift.png";
}

/**
 * Endpoint selection.
 *
 * Mirrors Corsair_Elite_Capellix_LCD.js and Platinum_Pro_XT_Elite_AIO.js, which both use
 * `endpoint.interface === -1 || endpoint.interface === 0`; -1 is the non-composite case.
 *
 * Worth knowing: the shipped Lian_Li_Universal_Screen_88.js (same vendor, same protocol)
 * exports no Validate at all and lets SignalRGB take the default. If this device fails to
 * bind on the bench, deleting this function is the first thing to try - it can only ever
 * reject an endpoint, never find a new one.
 */
export function Validate(endpoint) {
    return endpoint.interface === -1 || endpoint.interface === 0;
}

// ---------------------------------------------------------------------------------------
// User-settable parameters
// ---------------------------------------------------------------------------------------
// SignalRGB injects each `property` into the plugin scope as a variable of the same name,
// so `screenBrightness`, `targetFps` and the rest are read through readParam() below, which
// tolerates them being absent (before SignalRGB has set them) and being delivered as
// strings (which is how the defaults come back).
//
// None of them may be called "brightness": SignalRGB owns that name. It has a built-in
// per-device brightness slider with its own onBrightnessChanged() callback and a
// device.getBrightness() reader, and no plugin shipping with 2.5.74 declares a parameter by
// that name.

/* global
screenBrightness:readonly
targetFps:readonly
screenRotation:readonly
pushMode:readonly
readAcks:readonly
ringEnabled:readonly
ringBrightness:readonly
ringReverse:readonly
ringOffset:readonly
ringMode:readonly
ringWaitForAck:readonly
ringMinGapMs:readonly
ringRefreshS:readonly
ringBatchFrames:readonly
ringSampleMs:readonly
*/
export function ControllableParameters() {
    return [
        {
            property: "screenBrightness",
            group: "screen",
            label: "Screen Brightness",
            type: "number",
            step: "5",
            min: "0",
            max: "100",
            default: "80"
        },
        {
            property: "targetFps",
            group: "screen",
            label: "Frames per second",
            type: "number",
            step: "1",
            min: "1",
            max: "60",
            default: "30"
        },
        {
            property: "screenRotation",
            group: "screen",
            label: "Rotation",
            type: "combobox",
            values: ["0", "90", "180", "270"],
            default: "0"
        },
        {
            property: "ringEnabled",
            group: "lighting",
            label: "Ring Lighting",
            type: "boolean",
            default: "true"
        },
        {
            property: "ringBrightness",
            group: "lighting",
            label: "Ring Brightness",
            type: "number",
            step: "5",
            min: "0",
            max: "100",
            default: "100"
        },
        {
            property: "ringMode",
            group: "lighting",
            label: "Ring Push Mode",
            description: "Static pushes one frame whenever the colour changes, the only cadence proven on this block. Batch uploads a whole animation for the block to play by itself and is UNTESTED on hardware.",
            type: "combobox",
            values: ["Static", "Batch"],
            default: "Static"
        },
        {
            property: "ringReverse",
            group: "lighting",
            label: "Reverse Ring Direction",
            type: "boolean",
            default: "false"
        },
        {
            property: "ringOffset",
            group: "lighting",
            label: "Ring Start Offset",
            description: "Rotates the effect around the ring. Which physical LED is index 0 has never been read off the hardware, so this is how it gets corrected.",
            type: "number",
            step: "1",
            min: "0",
            max: "23",
            default: "0"
        },
        {
            property: "ringMinGapMs",
            group: "advanced",
            label: "Ring Minimum Gap (ms)",
            description: "Static mode never pushes two ring frames closer together than this. Colour changes inside the gap are coalesced and the latest colours go out when it elapses.",
            type: "number",
            step: "50",
            min: "34",
            max: "5000",
            default: "100"
        },
        {
            property: "ringRefreshS",
            group: "advanced",
            label: "Ring Refresh (s)",
            description: "Re-push the current ring frame every N seconds even when nothing changed. 0 is off. Only needed if the block turns out to drop its lighting state when idle.",
            type: "number",
            step: "1",
            min: "0",
            max: "60",
            default: "0"
        },
        {
            property: "ringBatchFrames",
            group: "advanced",
            label: "Ring Batch Frames",
            description: "Batch mode only: how many frames ride in one upload.",
            type: "number",
            step: "1",
            min: "2",
            max: "60",
            default: "24"
        },
        {
            property: "ringSampleMs",
            group: "advanced",
            label: "Ring Sample Interval (ms)",
            description: "Batch mode only: how often the canvas is sampled, and the interval the block is told to play the uploaded frames at. One byte on the wire, so 255 is the ceiling.",
            type: "number",
            step: "5",
            min: "20",
            max: "255",
            default: "100"
        },
        {
            property: "pushMode",
            group: "advanced",
            label: "Frame push mode",
            type: "combobox",
            values: ["Single write", "Chunked 1016"],
            default: "Single write"
        },
        {
            property: "ringWaitForAck",
            group: "advanced",
            label: "Ring: wait for each acknowledgement",
            description: "Off: ring pushes are fire-and-forget so the screen never waits on them (default). On: each ring push waits about 50 ms for the block's acknowledgement, which guarantees it is processed at the cost of screen frame rate.",
            type: "boolean",
            default: "false"
        },
        {
            property: "readAcks",
            group: "advanced",
            label: "Read acknowledgements",
            type: "boolean",
            default: "1"
        }
    ];
}

// ---------------------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------------------

/** Bulk OUT. Confirmed from Lian_Li_Universal_Screen_88.js. */
const ENDPOINT_OUT = 0x01;

/** Bulk IN. Confirmed from Lian_Li_Universal_Screen_88.js. */
const ENDPOINT_IN = 0x81;

const REPLY_LENGTH = 512;
const COMMAND_TIMEOUT_MS = 5000;
const FRAME_TIMEOUT_MS = 1000;
const ACK_TIMEOUT_MS = 30;   // the block acks a push in about 7 ms; 100 ms stalled Render

/** The drain: a short timeout, because a read that finds nothing is the answer we want. */
const DRAIN_TIMEOUT_MS = 50;
const DRAIN_MAX_READS = 32;

/** The clock display mode the HydroShift II init sequence uses (PUMP-CONTROL.md, s7). */
const CLOCK_MODE = 2;

/** The panel's own refresh rate, set once at init. Not the plugin's push rate. */
const PANEL_FRAME_RATE = 30;

/** What the block reports in ack byte 8 before the host should back off. */
const BUFFER_LEVEL_HIGH_WATER = 3;
const BUFFER_LEVEL_LOW_WATER = 2;

/** Corsair_Elite_Capellix_LCD.js chunks at this size; used only in "Chunked 1016" mode. */
const CHUNK_SIZE = 1016;

/**
 * SignalRGB calls Render on the clock setFrameRateTarget(fps) sets. Gating a frame on
 * `now - lastFrameAt >= 1000 / fps` inside a loop ticking at exactly 1000 / fps is a double
 * throttle: a tick that lands a millisecond early is rejected and the frame waits a whole
 * extra tick, halving the real rate. The slack turns the gate into a floor.
 */

/**
 * A real 480x480 all-black baseline JPEG (4:2:0, quality 25, 4226 bytes). Pushed to clear
 * the background layer at Initialize and again at Shutdown. Generated on this machine with
 * System.Drawing rather than hand-rolled, so it is an encoder-produced file the firmware
 * has the best chance of accepting. To regenerate:
 *
 *   Add-Type -AssemblyName System.Drawing
 *   $bmp = New-Object System.Drawing.Bitmap 480,480
 *   $g = [System.Drawing.Graphics]::FromImage($bmp); $g.Clear([System.Drawing.Color]::Black); $g.Dispose()
 *   ... save as JPEG, hex-encode the bytes ...
 */
const BLACK_JPEG_HEX =
    "ffd8ffe000104a46494600010101006000600000ffdb0043002016181c1814201c1a1c24222026305034302c2c306246" +
    "4a3a5074667a787266706e8090b89c8088ae8a6e70a0daa2aebec4ced0ce7c9ae2f2e0c8f0b8cacec6ffdb0043012224" +
    "24302a305e34345ec6847084c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6c6" +
    "c6c6c6c6c6c6c6c6c6c6c6c6c6c6ffc000110801e001e003012200021101031101ffc4001f0000010501010101010100" +
    "000000000000000102030405060708090a0bffc400b5100002010303020403050504040000017d010203000411051221" +
    "31410613516107227114328191a1082342b1c11552d1f02433627282090a161718191a25262728292a3435363738393a" +
    "434445464748494a535455565758595a636465666768696a737475767778797a838485868788898a9293949596979899" +
    "9aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1" +
    "f2f3f4f5f6f7f8f9faffc4001f0100030101010101010101010000000000000102030405060708090a0bffc400b51100" +
    "020102040403040705040400010277000102031104052131061241510761711322328108144291a1b1c109233352f015" +
    "6272d10a162434e125f11718191a262728292a35363738393a434445464748494a535455565758595a63646566676869" +
    "6a737475767778797a82838485868788898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4" +
    "c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9faffda000c03010002110311003f00e7" +
    "e8a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "28a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a0028a28a00" +
    "ffd9";

let blackJpeg = null;

// ---------------------------------------------------------------------------------------
// The RGB ring: geometry
// ---------------------------------------------------------------------------------------
// 24 LEDs, one zone (..\docs\ring-protocol.md), laid out on the outer circle of a 13 x 13
// grid so the middle 7 x 7 stays clear of the LCD image. Identical geometry to
// LianLi_HydroShift2_LCD_Host.js and the bridge plugin, on purpose: all three declare the
// same physical device and an effect must land the same way on any of them.

const RING_LED_COUNT = 24;
const RING_GRID = 13;
const RING_RADIUS = 6;
const RING_SUBDEVICE = "Ring";
const RING_FRAME_BYTES = RING_LED_COUNT * 3;   // 72

/**
 * The trailer interval for a single-frame push. The reference hard-codes 100 at both of its
 * call sites and that is the shape proven on this hardware (ring-protocol.md). Batch mode
 * sends ringSampleMs instead, because there the interval is the playback rate.
 */
// 20, not 100. ring characterize (2026-09-01) pushed single frames at 10 fps with a 20 ms
// trailer and every push was acknowledged after about 50 ms of processing, while the earlier
// 10 fps run with a 100 ms trailer showed only its first frame: the block ignores a new push
// while it is still holding the previous frame for its declared interval.
const RING_STATIC_INTERVAL_MS = 20;


function buildRingPositions() {
    const centre = (RING_GRID - 1) / 2;
    const positions = [];

    for (let i = 0; i < RING_LED_COUNT; i++) {
        const angle = -Math.PI / 2 + (2 * Math.PI * i) / RING_LED_COUNT;

        positions.push([
            Math.round(centre + RING_RADIUS * Math.cos(angle)),
            Math.round(centre + RING_RADIUS * Math.sin(angle))
        ]);
    }

    return positions;
}

const RING_POSITIONS = buildRingPositions();
const RING_NAMES = RING_POSITIONS.map(function (_, i) { return "Ring " + (i + 1); });

// ---------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------

/**
 * One timestamp source per connection. The firmware requires frame bytes 4..7 to increase
 * strictly from one frame to the next, and this is what guarantees it.
 */
let timestamps = null;

/** The transport object Initialize built. See the swap point note in the file header. */
let transport = null;

let lastFramePushedAt = 0;
let lastBrightnessSent = -1;
let lastRotationSent = -1;
let initialised = false;
let ackFailureLogged = false;

/** Ring state. lastRingFrame is the 72 bytes actually on the wire, not the last read. */
let lastRingFrame = null;
let lastRingPushAt = 0;
let ringDirty = false;
let ringBlanked = false;
let ringUnavailableLogged = false;
let ringApiMissingLogged = false;

/** Batch mode: frames collected so far, and the quiet window after an upload. */
let ringBatch = [];
let ringLastSampleAt = 0;
let ringQuietUntil = 0;

// ---------------------------------------------------------------------------------------
// Parameter helpers
// ---------------------------------------------------------------------------------------

/** Reads an injected parameter, tolerating "not set yet" and string-typed values. */
function readParam(value, fallback) {
    if (value === undefined || value === null || value === "") return fallback;
    return value;
}

function clamp(value, low, high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

function paramNumber(value, fallback, low, high) {
    let n = Number(readParam(value, fallback));
    if (isNaN(n)) n = Number(fallback);
    return Math.round(clamp(n, low, high));
}

function paramBoolean(value, fallback) {
    const v = readParam(value, fallback);
    return v === true || v === "true" || v === 1 || v === "1";
}

function currentBrightness() {
    return paramNumber(
        typeof screenBrightness !== "undefined" ? screenBrightness : undefined, 80, 0, 100);
}

function currentFps() {
    return paramNumber(typeof targetFps !== "undefined" ? targetFps : undefined, 30, 1, 60);
}

function currentRotationStep() {
    // The combobox offers degrees; the firmware wants 0..3.
    const degrees = paramNumber(
        typeof screenRotation !== "undefined" ? screenRotation : undefined, 0, 0, 270);
    return Math.round(degrees / 90) & 0x03;
}

function currentPushMode() {
    return String(readParam(typeof pushMode !== "undefined" ? pushMode : undefined,
        "Single write"));
}

function currentReadAcks() {
    return paramBoolean(typeof readAcks !== "undefined" ? readAcks : undefined, true);
}

function ringIsEnabled() {
    return paramBoolean(typeof ringEnabled !== "undefined" ? ringEnabled : undefined, true);
}

function ringIsReversed() {
    return paramBoolean(typeof ringReverse !== "undefined" ? ringReverse : undefined, false);
}

function currentRingBrightness() {
    return paramNumber(
        typeof ringBrightness !== "undefined" ? ringBrightness : undefined, 100, 0, 100);
}

function currentRingOffset() {
    return paramNumber(
        typeof ringOffset !== "undefined" ? ringOffset : undefined, 0, 0, RING_LED_COUNT - 1);
}

function currentRingMode() {
    return String(readParam(typeof ringMode !== "undefined" ? ringMode : undefined, "Static"));
}

function currentRingMinGapMs() {
    return paramNumber(
        typeof ringMinGapMs !== "undefined" ? ringMinGapMs : undefined, 100, 34, 5000);
}

function currentRingRefreshMs() {
    return paramNumber(
        typeof ringRefreshS !== "undefined" ? ringRefreshS : undefined, 0, 0, 60) * 1000;
}

function currentRingBatchFrames() {
    return paramNumber(
        typeof ringBatchFrames !== "undefined" ? ringBatchFrames : undefined, 24, 2, 60);
}

function currentRingSampleMs() {
    return paramNumber(
        typeof ringSampleMs !== "undefined" ? ringSampleMs : undefined, 100, 20, 255);
}

// ---------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------

function sameBytes(a, b) {
    if (!a || !b || a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
    return true;
}

// Diagnostics mirror: SignalRGB keeps device.log() output in an in-app console that no file
// ever sees, so every line is also sent as plain text to 127.0.0.1:DIAG_PORT where a local
// listener can record it. Nothing listens there in normal use and UDP does not care.
const DIAG_PORT = 48213;
let diagSocket = null;

function mirror(text) {
    try {
        if (typeof udp === "undefined" || !udp) return;
        if (!diagSocket) diagSocket = udp.createSocket();
        const out = [];
        for (let i = 0; i < text.length && out.length < 1400; i++) {
            const c = text.charCodeAt(i);
            out.push(c >= 32 && c < 127 ? c : 63);
        }
        diagSocket.write(out, "127.0.0.1", DIAG_PORT);
    } catch (e) { /* diagnostics never break the plugin */ }
}

function log(message) {
    if (typeof device !== "undefined" && device && typeof device.log === "function") {
        device.log("HydroShift II: " + message);
    }
    mirror("HS2LCD " + message);
}

// Runs one Initialize step, naming it before and naming it again if it throws, so a bare
// "TypeError: Type error" from a native call (all SignalRGB's log shows) can be placed.
function step(name, action) {
    mirror("HS2LCD init: " + name);
    try {
        return action();
    } catch (e) {
        log("init FAILED at " + name + ": " + e);
        throw e;
    }
}

function pause(ms) {
    if (typeof device !== "undefined" && device && typeof device.pause === "function") {
        device.pause(ms);
    }
}

// =======================================================================================
// TRANSPORT - THE SWAP POINT
// =======================================================================================
// This factory is the only place in the file that touches the USB endpoints, and the only
// place that names the backend transfer call at all. Everything above and below it works
// through the object it returns, so a different carrier is one more factory of the same
// shape and nothing else. lcd-plugin.test.js greps the source to keep it that way.

function createUsbTransport(dev) {
    // The call signature is device.bulk_transfer(endpointAddress, dataArray, length,
    // timeoutMs), confirmed from Lian_Li_Universal_Screen_88.js, Nzxt_Kraken_Z3_AIO.js and
    // ASUS_Ryujin_AIO_Controller.js; ASUS_Ryuo_AIO_LCD_Controller.js calls it with the
    // timeout omitted, so that argument is optional. Reads use the IN address as the first
    // argument, the same call. The pre-filled `new Array(512).fill(0)` read buffer is copied
    // verbatim from Lian_Li_Universal_Screen_88.js, whose comment explains it: "Bulk packets
    // require a predefined array to read properly. Backend probably isn't padding it out
    // like we do for HID."
    return {
        name: "usb",

        /** One bulk write. Returns whatever the backend returns; callers ignore it. */
        write: function (bytes, timeoutMs) {
            return dev.bulk_transfer(ENDPOINT_OUT, bytes, bytes.length,
                timeoutMs === undefined ? COMMAND_TIMEOUT_MS : timeoutMs);
        },

        /** One bulk read. Returns the reply array, or something falsy on a timeout. */
        read: function (length, timeoutMs) {
            const want = length || REPLY_LENGTH;

            return dev.bulk_transfer(ENDPOINT_IN, new Array(want).fill(0), want,
                timeoutMs === undefined ? ACK_TIMEOUT_MS : timeoutMs);
        },

        /**
         * Clears replies a previous owner of the handle left queued. L-Connect, the bench
         * tool or a previous SignalRGB session can all leave the IN endpoint holding acks
         * for commands this plugin never sent; reading them as if they answered our own
         * frames throws the back-pressure logic off by one for the whole session. Reads
         * until a read comes back empty or throws, at most DRAIN_MAX_READS times. Returns
         * how many stale replies were thrown away.
         */
        drain: function () {
            let drained = 0;

            for (let i = 0; i < DRAIN_MAX_READS; i++) {
                let stale = null;

                try {
                    stale = dev.bulk_transfer(ENDPOINT_IN, new Array(REPLY_LENGTH).fill(0),
                        REPLY_LENGTH, DRAIN_TIMEOUT_MS);
                } catch (e) {
                    return drained;
                }

                if (!stale || stale.length === 0) return drained;

                drained++;
            }

            return drained;
        }
    };
}

// =======================================================================================
// End of the transport. Nothing below here names an endpoint or a transfer call.
// =======================================================================================

/**
 * Reads one 512-byte reply and returns it, or null.
 *
 * The device answers every command with a 512-byte plaintext reply. The retry-with-pause
 * loop is copied from Lian_Li_Universal_Screen_88.js.
 */
function readReply(tp, attempts, expectedOpcode) {
    if (!tp || !currentReadAcks()) return null;

    const tries = attempts || 5;

    for (let i = 0; i < tries; i++) {
        try {
            const reply = tp.read(REPLY_LENGTH, ACK_TIMEOUT_MS);

            if (reply && reply.length > 0) {
                // Replies echo their opcode in byte 0. A ring ack (0xFC) that arrives while a
                // screen push is waiting is not the screen's ack: skip it and keep reading.
                if (expectedOpcode !== undefined && (reply[0] & 0xFF) !== expectedOpcode) continue;

                return reply;
            }
        } catch (e) {
        }

        pause(2);
    }

    if (!ackFailureLogged) {
        log("no reply on the IN endpoint after " + tries
            + " attempts. Continuing fire-and-forget; frames are still being written.");
        ackFailureLogged = true;
    }

    return null;
}
/**
 * Back-pressure. PUMP-CONTROL.md section 7: if ack byte 8 is above 3, poll QueryBlock every
 * 50 ms until the block's frame buffer level falls to 2 or less.
 */

/** Writes one already-built stream, whole or in 1016-byte chunks per the pushMode setting. */
function writeStream(tp, stream, timeoutMs) {
    if (currentPushMode() === "Chunked 1016" && stream.length > CHUNK_SIZE) {
        const chunks = HS2.chunkStream(stream, CHUNK_SIZE);

        for (let i = 0; i < chunks.length; i++) tp.write(chunks[i], timeoutMs);

        return;
    }

    tp.write(stream, timeoutMs);
}

/** Sends a 512-byte command frame and drains its reply. */
function sendCommand(tp, frame) {
    tp.write(frame, COMMAND_TIMEOUT_MS);

    return readReply(tp);
}

// ---------------------------------------------------------------------------------------
// Image push
// ---------------------------------------------------------------------------------------

/**
 * Grabs the SignalRGB canvas as a JPEG.
 *
 * LCD.getFrame({format: "JPEG"}) after LCD.initialize({width, height}) is what every LCD
 * plugin that ships with SignalRGB uses, including Lian_Li_Universal_Screen_88.js, and it
 * hands back exactly the JPEG-encoded canvas at the initialised size. That is the primary
 * path here.
 *
 * device.getImageBuffer(x0, y0, x1, y1, {flipH, outputWidth, outputHeight, format}) is the
 * lower-level alternative - the only shipped plugin using it is Corsair_Nexus_Screen.js,
 * and it asks for "BMP" there, not "JPEG". It is kept as a fallback for a runtime where
 * @SignalRGB/lcd is unavailable, but LCD.getFrame is the confirmed-good route and should
 * be the one that runs.
 */
function grabJpegFrame() {
    if (typeof LCD !== "undefined" && LCD && typeof LCD.getFrame === "function") {
        return LCD.getFrame({ format: "JPEG" });
    }

    if (typeof device !== "undefined" && device && typeof device.getImageBuffer === "function") {
        return device.getImageBuffer(0, 0, HS2.PanelInfo.Width - 1, HS2.PanelInfo.Height - 1, {
            flipH: false,
            outputWidth: HS2.PanelInfo.Width,
            outputHeight: HS2.PanelInfo.Height,
            format: "JPEG"
        });
    }

    return null;
}

/**
 * Pushes one JPEG to the background layer: the PushJpg (0x65) header frame carrying the
 * length as a big-endian uint32, immediately followed by the JPEG bytes.
 *
 * BENCH QUESTION 1 - DOES THE FIRMWARE ACCEPT A PUSH SPLIT ACROSS BULK WRITES?
 * -----------------------------------------------------------------------------------
 * Unsettled, and the "Chunked 1016" mode is the thing that has to be proved or discarded on
 * hardware.
 *
 * What is known: the Linux reference driver sends header + JPEG as ONE write, and so does
 * the shipped Lian_Li_Universal_Screen_88.js, which writes 512 + jpegLength bytes in a
 * single call for a 480x1920 panel whose frames are far larger than 1025 bytes. That is
 * direct evidence from a working, shipping SignalRGB plugin on this same vendor that a
 * single oversized write is both permitted by SignalRGB and accepted by Lian Li firmware,
 * and it contradicts the 1025-byte cap noted in PUMP-CONTROL.md section 8. So "Single write"
 * is the default here.
 *
 * What is NOT known: whether THIS firmware (0xA034) tolerates the alternative - the same
 * stream cut into <=1016-byte pieces across several calls. That is what "Chunked 1016"
 * exercises. Do not assume either mode works until a frame has visibly landed on the panel.
 */
function pushJpeg(tp, jpegBytes) {
    if (!tp || !jpegBytes || jpegBytes.length === 0) return;

    if (jpegBytes.length > HS2.PanelInfo.MaxPayloadBytes) {
        log("frame is " + jpegBytes.length + " bytes, over the "
            + HS2.PanelInfo.MaxPayloadBytes + " byte cap. Dropped.");

        return;
    }

    writeStream(tp, HS2.buildJpgPushStream(timestamps, jpegBytes), FRAME_TIMEOUT_MS);

    // Block until the panel acknowledges this frame (up to about 1.6 s), exactly as the
    // shipped Universal Screen 88 plugin does; a late ring acknowledgement (0xFC) in the
    // pipe is skipped by opcode. Nothing else paces the screen.
    readReply(tp, 50, HS2.Opcode.PushJpg);
}function pushBlackFrame(tp) {
    if (!blackJpeg) blackJpeg = HS2.hexToBytes(BLACK_JPEG_HEX);

    pushJpeg(tp, blackJpeg);
}

// ---------------------------------------------------------------------------------------
// The RGB ring
// ---------------------------------------------------------------------------------------
// One command, PushRgbData (0xFC): a 512-byte header whose parameter block carries the
// payload length big-endian at parameter offset 4, then a tinyuz-compressed colour block and
// a four-byte trailer of [frames hi, frames lo, interval ms, led count]. The whole thing is
// built by HS2.ring.pushStream, which is the only piece of the ring protocol this file
// depends on; the geometry, the index mapping and the brightness scaling are done here
// because they are canvas concerns, not wire concerns. There is no ring-brightness opcode -
// the reference scales the colours on the host and so does this (ring-protocol.md).

function ringApiAvailable() {
    if (HS2 && HS2.ring && typeof HS2.ring.pushStream === "function") return true;

    if (!ringApiMissingLogged) {
        log("the embedded protocol module has no ring support; the RGB ring stays off the "
            + "wire and the screen carries on.");
        ringApiMissingLogged = true;
    }

    return false;
}

function offRingFrame() {
    const frame = new Array(RING_FRAME_BYTES);

    for (let i = 0; i < RING_FRAME_BYTES; i++) frame[i] = 0;

    return frame;
}

/**
 * Reads the 24 subdevice colours and flattens them to 72 bytes of R, G, B in ring order.
 *
 * device.subdeviceColor("Ring", x, y) is the Corsair_XC7_LCD.js:259 call shape. Three
 * mappings are applied on the way out, all of them host-side and all of them reversible from
 * the UI, because which physical LED is index 0 and which way the indices run have never
 * been read off this hardware:
 *
 *   ringBrightness  scales every channel (there is no ring-brightness command)
 *   ringReverse     index i takes the colour of (24 - i) % 24, which keeps Ring 1 in place
 *   ringOffset      then rotates the whole thing by 0..23 LEDs
 */
function readRingFrame() {
    if (typeof device === "undefined" || !device || typeof device.subdeviceColor !== "function") {
        if (!ringUnavailableLogged) {
            log("device.subdeviceColor is not available; the RGB ring stays off the wire.");
            ringUnavailableLogged = true;
        }

        return null;
    }

    const scale = currentRingBrightness() / 100;
    const reversed = ringIsReversed();
    const offset = currentRingOffset();
    const frame = new Array(RING_FRAME_BYTES);

    for (let i = 0; i < RING_LED_COUNT; i++) {
        const spun = reversed ? (RING_LED_COUNT - i) % RING_LED_COUNT : i;
        const source = (spun + offset) % RING_LED_COUNT;
        const position = RING_POSITIONS[source];
        const colour = device.subdeviceColor(RING_SUBDEVICE, position[0], position[1])
            || [0, 0, 0];

        frame[i * 3] = clamp(Math.round(colour[0] * scale), 0, 255);
        frame[i * 3 + 1] = clamp(Math.round(colour[1] * scale), 0, 255);
        frame[i * 3 + 2] = clamp(Math.round(colour[2] * scale), 0, 255);
    }

    return frame;
}

/**
 * One PushRgbData for the given frames, then its ack read - the same discipline the screen
 * uses. `frames` is an array of 72-byte arrays; one of them is the static case.
 */
function currentRingWaitForAck() {
    return paramBoolean(typeof ringWaitForAck !== "undefined" ? ringWaitForAck : undefined, false);
}

function pushRing(tp, frames, intervalMs) {
    if (!tp || !ringApiAvailable() || !frames || frames.length === 0) return false;

    writeStream(tp, HS2.ring.pushStream(timestamps, frames, intervalMs), FRAME_TIMEOUT_MS);

    if (currentRingWaitForAck()) {
        readReply(tp, 5, HS2.Opcode.PushRgbData);

        return true;
    }

    // Fire and forget. The block takes about 50 ms to acknowledge a ring push; waiting for it
    // here stalled every screen frame in between. The ack is consumed and discarded by the
    // next opcode-aware readReply on the screen path (or by the drain at the next open).
    return true;
}
function resetRingBatch() {
    ringBatch = [];
    ringLastSampleAt = 0;
    ringQuietUntil = 0;
}

/**
 * Static: the reference's own cadence, and the only one proven on this block. One frame,
 * trailer interval 100, sent when the colours change and not otherwise. Two extra rules on
 * top of the reference, both because of the 10 fps chase that the block ignored:
 *
 *   - never two pushes closer together than ringMinGapMs
 *   - a change inside that gap is remembered, not queued; when the gap elapses the LATEST
 *     colours go out, once
 */
function renderRingStatic(tp, now) {
    const frame = readRingFrame();

    if (!frame) return;

    if (!sameBytes(frame, lastRingFrame)) ringDirty = true;

    const refreshMs = currentRingRefreshMs();
    const refreshDue = refreshMs > 0 && lastRingFrame !== null
        && (now - lastRingPushAt) >= refreshMs;

    if (!ringDirty && !refreshDue) return;

    // The first push of a session goes out immediately; after that the gap applies and a
    // still-dirty frame simply waits for the next tick.
    if (lastRingFrame !== null && (now - lastRingPushAt) < currentRingMinGapMs()) return;

    if (!pushRing(tp, [frame], RING_STATIC_INTERVAL_MS)) return;

    lastRingFrame = frame;
    lastRingPushAt = now;
    ringDirty = false;
}

/**
 * Batch: sample the canvas every ringSampleMs, and when ringBatchFrames have been collected
 * hand the block the whole animation in ONE upload with the sample period as the trailer
 * interval, then say nothing at all for frames x interval ms while it plays.
 *
 * UNTESTED ON HARDWARE. The reference has never sent a multi-frame push over the wire; its
 * doc comment for the RF equivalent says the firmware "loops the animation onboard at
 * interval_ms with no further host packets", which is the behaviour this mode assumes.
 */
function renderRingBatch(tp, now) {
    if (now < ringQuietUntil) return;

    const sampleMs = currentRingSampleMs();

    if (ringBatch.length > 0 && (now - ringLastSampleAt) < sampleMs) return;

    const frame = readRingFrame();

    if (!frame) return;

    ringBatch.push(frame);
    ringLastSampleAt = now;

    const wanted = currentRingBatchFrames();

    if (ringBatch.length < wanted) return;

    const frames = ringBatch;

    ringBatch = [];

    if (!pushRing(tp, frames, sampleMs)) return;

    lastRingFrame = frames[frames.length - 1];
    lastRingPushAt = now;
    ringDirty = false;
    ringQuietUntil = now + wanted * sampleMs;
}

function renderRing(tp, now) {
    if (!tp || !ringApiAvailable()) return;

    if (!ringIsEnabled()) {
        if (ringBlanked) return;

        if (!pushRing(tp, [offRingFrame()], RING_STATIC_INTERVAL_MS)) return;

        lastRingFrame = offRingFrame();
        lastRingPushAt = now;
        ringDirty = false;
        ringBlanked = true;
        resetRingBatch();

        return;
    }

    if (ringBlanked) {
        // Just re-enabled: forget the black frame so the live colours go out on this tick.
        ringBlanked = false;
        lastRingFrame = null;
        lastRingPushAt = 0;
        ringDirty = true;
        resetRingBatch();
    }

    if (currentRingMode() === "Batch") {
        renderRingBatch(tp, now);

        return;
    }

    renderRingStatic(tp, now);
}

/** Corsair_XC7_LCD.js:236-245, call for call and in the same order. */
function initRing() {
    if (typeof device === "undefined" || !device) return;

    if (typeof device.createSubdevice === "function") device.createSubdevice(RING_SUBDEVICE);

    if (typeof device.setSubdeviceLeds === "function") {
        device.setSubdeviceLeds(RING_SUBDEVICE, RING_NAMES.slice(),
            RING_POSITIONS.map(function (p) { return p.slice(); }));
    }

    if (typeof device.setSubdeviceName === "function") {
        device.setSubdeviceName(RING_SUBDEVICE, "Block Ring");
    }

    if (typeof device.setSubdeviceImageUrl === "function") {
        device.setSubdeviceImageUrl(RING_SUBDEVICE,
            "https://assets.signalrgb.com/devices/brands/lian-li/aio/hydroshift.png");
    }

    if (typeof device.setSubdeviceSize === "function") {
        device.setSubdeviceSize(RING_SUBDEVICE, RING_GRID, RING_GRID);
    }
}

// ---------------------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------------------

/**
 * BENCH QUESTION 2 - CAN THE DEVICE BE OPENED WHILE THE FANCONTROL PLUGIN HOLDS IT?
 * -----------------------------------------------------------------------------------
 * Unsettled, and it decides whether this plugin exists at all (PUMP-CONTROL.md section 8,
 * "the exclusivity branch"). WinUSB handles are commonly exclusive per interface. If
 * SignalRGB and the FanControl plugin cannot both hold 1CBE:A034, this plugin is dropped
 * and the screen moves into the FanControl plugin instead.
 *
 * Nothing in this file can detect that condition reliably - a failure here looks like any
 * other open failure. Test it directly on the bench: start FanControl with its plugin
 * loaded, then start SignalRGB, and see whether both keep working. Test it in both orders.
 */
export function Initialize() {
    timestamps = new HS2.MonotonicTimestampSource();
    lastBrightnessSent = -1;
    lastRotationSent = -1;
    ackFailureLogged = false;
    lastFramePushedAt = 0;

    lastRingFrame = null;
    lastRingPushAt = 0;
    ringDirty = false;
    ringBlanked = false;
    ringUnavailableLogged = false;
    ringApiMissingLogged = false;
    resetRingBatch();

    // 480x480 square panel (PanelInfo / ScreenInfo::HYDROSHIFT2). Lian_Li_Universal_Screen_88.js
    // carries a warning that its panel will not accept any size but its native one, so this
    // is deliberately the panel's exact resolution. No `circular: true` either: the LCD-S is
    // the square-screen model (h2_aio.rs:72 sets is_square for PID 0xA034), and the round
    // Corsair_XC7_LCD.js is the only shipped plugin that passes that flag.
    step("LCD.initialize", function () {
        LCD.initialize({ width: HS2.PanelInfo.Width, height: HS2.PanelInfo.Height });
    });

    transport = step("createUsbTransport", function () { return createUsbTransport(device); });

    // Anything the previous owner of this handle left queued on the IN endpoint is not an
    // answer to us. Throw it away before the first command so the acks line up.
    const stale = step("drain", function () { return transport.drain(); });

    if (stale > 0) log("discarded " + stale + " stale reply/replies left on the endpoint.");

    // Wake preamble: StopPlay, StopClock, GetVer, each its own write and read, 150 ms apart.
    // After the LCD has been in play mode the block ignores control commands until this
    // re-arms the channel (PUMP-CONTROL.md section 3 / Commands.WakePreamble).
    step("wake preamble", function () {
        sendCommand(transport, HS2.commands.stopPlay(timestamps));
        pause(150);
        sendCommand(transport, HS2.commands.stopClock(timestamps));
        pause(150);
        sendCommand(transport, HS2.commands.getVer(timestamps));
        pause(150);
    });

    // Screen init (PUMP-CONTROL.md section 7): panel frame rate, clock, then stop the clock
    // so it does not draw over the pushed frames.
    step("screen init", function () {
        sendCommand(transport, HS2.commands.frameRate(timestamps, PANEL_FRAME_RATE));
        sendCommand(transport, HS2.commands.syncClock(timestamps, new Date(), CLOCK_MODE));
        sendCommand(transport, HS2.commands.stopClock(timestamps));
    });

    step("brightness", function () { applyBrightness(transport, true); });
    step("rotation", function () { applyRotation(transport, true); });

    // Clear both layers: drop the PNG overlay, then paint the background black so whatever
    // the offline preset left on screen is gone before the first real frame.
    step("clear layers", function () {
        sendCommand(transport, HS2.commands.clearPng(timestamps));
        pushBlackFrame(transport);
    });

    step("ring subdevice", function () { initRing(); });

    // Ask SignalRGB to call Render at roughly the configured rate. The manual clock in
    // Render() is still there because this is a target, not a guarantee.
    step("setFrameRateTarget", function () { device.setFrameRateTarget(currentFps()); });

    initialised = true;
    log("LCD-S 360 initialised at " + HS2.PanelInfo.Width + "x" + HS2.PanelInfo.Height + ", "
        + currentFps() + " fps, brightness " + currentBrightness() + ", ring "
        + (ringIsEnabled() ? currentRingMode().toLowerCase() : "off") + ".");
}

function applyBrightness(tp, force) {
    if (!tp) return;

    const value = currentBrightness();

    if (!force && value === lastBrightnessSent) return;

    sendCommand(tp, HS2.commands.brightness(timestamps, value));
    lastBrightnessSent = value;
}

function applyRotation(tp, force) {
    if (!tp) return;

    const value = currentRotationStep();

    if (!force && value === lastRotationSent) return;

    sendCommand(tp, HS2.commands.rotation(timestamps, value));
    lastRotationSent = value;
}

/**
 * Parameter change callbacks. SignalRGB calls on[property]Changed() before the next Render()
 * whenever the user moves a control. The name is the exact property name prefixed with "on"
 * and suffixed with "Changed", case sensitive, which is why the casing below looks wrong but
 * is not: the shipped plugins do the same (ondpiStagesChanged, onmoboSyncChanged and ~200
 * others all keep the first-letter casing of the property itself).
 *
 * The ones that are not here (ringBrightness, ringReverse, ringOffset, ringMinGapMs,
 * ringRefreshS, ringBatchFrames, ringSampleMs, pushMode, readAcks) need no callback: Render
 * re-reads every one of them on the next tick, and for the ring that tick also notices the
 * colours changed and pushes.
 */
export function onscreenBrightnessChanged() {
    if (initialised) applyBrightness(transport, false);
}

export function onscreenRotationChanged() {
    if (initialised) applyRotation(transport, false);
}

export function ontargetFpsChanged() {
    if (!initialised) return;

    device.setFrameRateTarget(currentFps());
    lastFramePushedAt = 0;
}

export function onringEnabledChanged() {
    if (!initialised) return;

    // Straight into renderRing: disabling sends the one black frame now, enabling resumes
    // now rather than on the next tick.
    renderRing(transport, Date.now());
}

export function onringModeChanged() {
    if (!initialised) return;

    resetRingBatch();
    ringDirty = true;
}

export function Render() {
    if (!initialised) return;

    // The shipped Lian Li plugins (Lian_Li_Universal_Screen_88.js, same DES protocol as this
    // block; Lian_Li_Galahad_II_LCD.js, ring plus 480x480 panel) do no pacing of their own:
    // every Render grabs the frame, writes it and blocks until the block acknowledges it. The
    // acknowledgement is the flow control, and SignalRGB's frame rate target is the clock.
    // 1.0.3 to 1.0.5 tried a frame clock, a skip-on-full hold and an adaptive rate on top of
    // that; each one fought the block's own pacing and showed up as judder.
    const now = Date.now();

    renderRing(transport, now);

    applyBrightness(transport, false);
    applyRotation(transport, false);

    const jpeg = grabJpegFrame();

    if (!jpeg || jpeg.length === 0) {
        log("canvas grab returned nothing; skipping this frame.");
        pause(1);

        return;
    }

    pushJpeg(transport, jpeg);
    pause(1);
}
export function Shutdown(SystemSuspending) {
    if (!initialised) return;

    try {
        // Black ring and black frame first, so the block is not left holding whatever
        // SignalRGB drew last, then stop playback. The block falls back to its offline preset
        // on its own once nothing is pushing; whichever plugin owns the screen has to re-push
        // at every start regardless (PUMP-CONTROL.md section 7).
        pushRing(transport, [offRingFrame()], RING_STATIC_INTERVAL_MS);
        pushBlackFrame(transport);
        sendCommand(transport, HS2.commands.stopPlay(timestamps));
    } catch (e) {
        log("shutdown push failed: " + e);
    }

    initialised = false;
    transport = null;
}
