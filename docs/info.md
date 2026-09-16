<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This tile is the RMSNorm/LayerNorm accelerator of a small-language-model
(SLM) inference SoC, packaged as a stand-alone TinyTapeout project. The full
SoC is too large for a single TinyTapeout slot, so it was split into
per-block tiles.

RMSNorm mode computes, over N ∈ {8, 16} signed INT8 inputs (the SoC block supports up to 64; this tile build depth-reduces the scratches to fit the tile budget):

- `ms = (sum x_i²) >> log2(N) + EPS`
- `r = round(32768 / sqrt(ms))` — a Q1.15 inverse square root, exactly
  rounded by a bit-serial search (no LUT, no Newton seed)
- `y_i = sat8((x_i · r · g_i) >>> (21 − out_shift))`, gamma g_i signed INT8
  interpreted as Q1.6

LayerNorm mode adds a mean pre-pass and normalizes `x_i − mu`. The result is
bit-defined — the included test checks outputs exactly, not within a
tolerance.

The block's own CSR interface (its SLB bus slave port) is exposed through a
32-bit SPI register bridge with the identical register map the SoC's CPU
uses: CTRL at 0x0000 (b0 start, b1 clear_done), STATUS at 0x0004 (b0 busy,
b1 done), CFG at 0x0008 ([6:0] N, b7 mode, b8 irq_en, [20:16] out_shift),
EPS at 0x000C, x scratch at 0x0100 (one INT8 per word, result overwrites
byte 0), gamma scratch at 0x0200. The SLB port could be bus-connected if
inter-tile wiring is available.

## How to test

1. Apply clock (25 MHz nominal; keep SCK below clk/8) and release `rst_n`
   with CS_n high.
2. SPI-write the N inputs at 0x0100+4i and gammas at 0x0200+4i, then CFG
   with N, mode, out_shift, and irq_en.
3. Write CTRL = 2, then CTRL = 1.
4. Poll STATUS until bit 1 (done) — or watch uio[0] (irq) — then read the
   normalized INT8 results back from 0x0100+4i byte 0.

The included cocotb test (`test/test.py`) runs an 8-element RMSNorm with
mixed-sign inputs and non-uniform gammas and checks all outputs bit-exactly,
including a bit-true Python model of the exactly-rounded inverse square root.

## External hardware

An SPI master (any MCU, or a Raspberry Pi's SPI pins) connected to
uio[4]=CS_n, uio[5]=SCK, uio[6]=MOSI, uio[3]=MISO. Optionally an LED/scope on
uio[0] (irq).
