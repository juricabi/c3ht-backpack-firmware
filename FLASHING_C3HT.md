# Building and flashing C3HT firmware with the ExpressLRS Configurator

This package is the C3HT firmware source, version **1.5.9-c3ht.1**: ExpressLRS Backpack 1.5.9 plus the C3HT
patches (head tracker, brownout handling, power and WiFi fixes). The ExpressLRS Configurator
builds it on your PC with your binding phrase and flashes it. This package contains firmware only, no board design.

## What you need

- **ExpressLRS Configurator** 1.8 or newer, with **git** installed (the Configurator needs it).
- This folder, unpacked to a **short path** such as `C:\c3ht\c3ht-1.5.9-c3ht.1`. Long paths break the build on Windows.
- The **binding phrase** of your radio. **Your radio's TX backpack must use the same phrase** (see Binding below).

## New board: first setup

A C3HT arrives with the standard firmware and no binding phrase. It starts in normal mode with the LED solid on.

1. **Start WiFi:** power-cycle the goggles 3 times. The LED blinks twice, pauses, and repeats (binding mode).
   After 2 minutes without a bind, the C3HT switches to WiFi by itself (LED flickers fast).
2. **Choose your network:** with a phone, join **ExpressLRS VRx Backpack** (password `expresslrs`) and open
   **http://10.0.0.1**. Pick your home network and enter its password. From then on, WiFi mode joins your home
   network and the C3HT is at `elrs_vrx.local`.
3. **Bind or flash:**
   - Bind from the radio's Lua during binding mode, or
   - flash your own build with your binding phrase as described below.

## Build and flash (Configurator, Backpack tab)

1. **Backpack**, then **LOCAL**, then **CHOOSE FOLDER**: pick this folder (the one with `platformio.ini`).
2. **Target**: choose the goggle or receiver, then **C3HT Head Tracker Backpack**:

   | Goggle / VRx | Device category | Device |
   |---|---|---|
   | Orqa (FPV.Connect) | Orqa Goggles FPV.Connect | C3HT Head Tracker Backpack |
   | ImmersionRC rapidFIRE | rapidFIRE Module | C3HT Head Tracker Backpack |
   | RX5808 single | RX5808 Single Module | C3HT Head Tracker Backpack |
   | RX5808 diversity | RX5808 Diversity Module | C3HT Head Tracker Backpack |
   | Skyzone SteadyView (SPI) | Skyzone Steadyview | C3HT Head Tracker Backpack, SPI (DIY) |
   | Skyzone goggles (MSP) | Skyzone Steadyview | C3HT Head Tracker Backpack, MSP (DIY) |
   | TBS Fusion | Fusion VRX | C3HT Head Tracker Backpack |

3. **Device options**: enter your **binding phrase**.
4. **Flashing method**:
   - **WiFi** for a board that already runs C3HT firmware. Put it into WiFi mode first (see below), then
     press **Build & Flash**. It uploads to `elrs_vrx.local`, so the PC and the C3HT must be on the same network.
     If the PC cannot find it, press **Build** only, then upload the resulting `firmware.bin` on the C3HT's web
     page (**Update**).
   - **UART** for a blank board (first flash, see below).
5. After flashing, the C3HT restarts. The LED is **solid on** in normal mode.

The firmware reports **1.5.9-c3ht.1** on its web page.

## Putting the C3HT into WiFi mode

Any of these works:

- **From the radio:** in the ExpressLRS Lua, open **WiFi Connectivity** and press **Enable VRx WiFi**. This only works
  once the C3HT and the radio's backpack share the binding phrase.
- **Power-cycles:** power-cycle the goggles **3 times**, leaving them on for about 3 s each time (at least 1.5 s
  counts), then leave them on. With a binding phrase compiled in, this starts WiFi. Without one, it starts binding mode.
- **BOOT:** a **short** press, under half a second (C3HT-Nano: briefly short the top pad **B** to the U.FL shell; first-batch boards and the NanoA: pads **B** and **G**).

WiFi mode: the LED flickers fast. The network is **ExpressLRS VRx Backpack** (password `expresslrs`), and the web page
is at **http://10.0.0.1**, or at **http://elrs_vrx.local** once you have saved your home network there.
WiFi mode ends after 5 minutes without a client.

## Binding

- **Binding phrase (recommended):** use the same phrase as your radio. **The radio's TX backpack needs the phrase
  as well.** Flash it in the Configurator's Backpack tab for your radio, with the same phrase. If the radio's backpack
  was flashed at the factory without a phrase, it sends nothing the C3HT listens to.
- **Without a phrase:**
  1. Power-cycle the goggles 3 times. The LED blinks twice, pauses, and repeats: binding mode for 2 minutes.
  2. Press **[Bind]** in the ExpressLRS Lua on the radio.
  3. If binding does not finish, your radio's backpack is not forwarding the bind. Use a phrase.
- **Check it works:** **Enable VRx WiFi** from the Lua must switch the C3HT into WiFi mode.

## First flash of a blank board (UART)

Use a USB-UART adapter set to **3.3 V logic**:

| Adapter | C3HT |
|---|---|
| TX | R |
| RX | T |
| VCC | + |
| GND | G |

1. Hold BOOT low while you apply power (C3HT-Nano: short the top pad **B** to the U.FL shell, or on first-batch
   boards and the NanoA the pads **B** and **G**, while plugging the adapter's USB in; C3HT-Mini: press the button). Release after about a second.
2. In the Configurator choose the **UART** flashing method and the adapter's COM port, then **Build & Flash**.
3. Power-cycle without BOOT.

On the FT232RL adapters with a 3.3/5 V jumper, **the jumper sets the logic level**. Leave it on 3.3 V while TX or RX are
connected. Their 3.3 V VCC pin is weak (about 50 mA), but that is enough for flashing.

## Head tracker

In the web page (WiFi mode), head-tracking page:

1. Set the **board orientation** to match how the board is mounted.
2. Run the **calibration** with the goggles still.
3. Save.

In the radio's Lua, Backpack folder, set the head-tracking switch and start channel.

## Safety

- **Orqa FPV.Connect slot:** the slot gives **3.3 V**. **Never feed 5 V into `+`** while the board is soldered to it.
  **Put insulating tape (Kapton) on the bottom of the C3HT-Nano** before you place it; its exposed test pads would
  otherwise touch the FPV.Connect. Photos of an install are in the [README](README.md#installing).
- The C3HT-Nano has **no reverse-polarity protection**. Check `+` and `G` before the first power-up.
- **Orqa:** enable **Ghost integration** in the goggle menu, then restart the goggles once; before that restart the
  goggles ignore band/channel changes. Stock Orqa firmware has no DVR control from the backpack.
