# SPDX-License-Identifier: Apache-2.0
# RMSNorm tile: run one RMSNorm pass over 8 signed INT8 inputs through the
# SPI register bridge and check the outputs bit-exactly against the spec:
#   ms  = (sum x_i^2 >> log2(N)) + eps
#   r   = round(32768 / sqrt(ms))          (exactly-rounded Q1.15)
#   y_i = sat8( (x_i * r * g_i) >>> (21 - out_shift) ),  gamma Q1.6
import math

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

CS, SCK, MOSI = 4, 5, 6
MISO = 3


def set_bit(v, b, x):
    return (v | (1 << b)) if x else (v & ~(1 << b))


async def spi_frame(dut, rw, addr, wdata=0):
    """One frame: 32-bit cmd {RW, 2'b10, 13'b0, addr[15:0]}, 32 data bits."""
    v = int(dut.uio_in.value)
    v = set_bit(v, CS, 0)
    v = set_bit(v, SCK, 0)
    dut.uio_in.value = v
    await ClockCycles(dut.clk, 6)
    cmd = (rw << 31) | (0b10 << 29) | (addr & 0xFFFF)
    rd = 0
    for i in range(31, -1, -1):          # command word
        v = set_bit(v, MOSI, (cmd >> i) & 1)
        v = set_bit(v, SCK, 0)
        dut.uio_in.value = v
        await ClockCycles(dut.clk, 4)
        v = set_bit(v, SCK, 1)
        dut.uio_in.value = v
        await ClockCycles(dut.clk, 4)
    await ClockCycles(dut.clk, 8)        # bridge SLB round-trip settle
    for i in range(31, -1, -1):          # data word
        v = set_bit(v, MOSI, (wdata >> i) & 1 if rw else 0)
        v = set_bit(v, SCK, 0)
        dut.uio_in.value = v
        await ClockCycles(dut.clk, 4)
        if not rw:
            rd |= ((int(dut.uio_out.value) >> MISO) & 1) << i
        v = set_bit(v, SCK, 1)
        dut.uio_in.value = v
        await ClockCycles(dut.clk, 4)
    v = set_bit(v, SCK, 0)
    dut.uio_in.value = v
    await ClockCycles(dut.clk, 4)
    v = set_bit(v, CS, 1)
    dut.uio_in.value = v
    await ClockCycles(dut.clk, 10)
    return rd


def sat8(v):
    return max(-128, min(127, v))


def invsqrt_q15(v):
    """Bit-exact model of the exactly-rounded Q1.15 inverse square root."""
    r = math.isqrt((1 << 30) // v)
    if (2 * r + 1) ** 2 * v <= (1 << 32):
        r += 1
    return r


def golden(xs, gs, osh, eps=1):
    n = len(xs)
    ms = (sum(x * x for x in xs) >> int(math.log2(n))) + eps
    r = invsqrt_q15(ms)
    return [sat8((x * r * g) >> (21 - osh)) for x, g in zip(xs, gs)]


@cocotb.test()
async def test_rmsnorm_tile(dut):
    dut._log.info("Start")
    clock = Clock(dut.clk, 40, units="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 1 << CS
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    xs = [10, -20, 3, 0, -128, 55, 7, -1]      # signed INT8 inputs
    gs = [64, 64, 32, 64, 64, 96, 64, -64]     # gamma, Q1.6 (64 = 1.0)
    n, osh = len(xs), 4

    for i, x in enumerate(xs):
        await spi_frame(dut, 1, 0x0100 + 4 * i, x & 0xFF)
    for i, g in enumerate(gs):
        await spi_frame(dut, 1, 0x0200 + 4 * i, g & 0xFF)
    # CFG: N[6:0], mode b7 = 0 (RMS), irq_en b8 = 1, out_shift [20:16]
    await spi_frame(dut, 1, 0x0008, (osh << 16) | (1 << 8) | n)
    # CTRL: clear_done then start
    await spi_frame(dut, 1, 0x0000, 2)
    await spi_frame(dut, 1, 0x0000, 1)

    status = 0
    for _ in range(60):
        status = await spi_frame(dut, 0, 0x0004)
        if status & 2:
            break
    assert status & 2, f"rmsnorm done never set (STATUS={status:#x})"
    assert ((int(dut.uio_out.value) >> 0) & 1) == 1, "irq not asserted"

    ref = golden(xs, gs, osh)
    for i in range(n):
        raw = (await spi_frame(dut, 0, 0x0100 + 4 * i)) & 0xFF
        y = raw - 256 if raw & 0x80 else raw
        assert y == ref[i], f"y[{i}]={y} vs bit-exact ref {ref[i]}"
    dut._log.info(f"rmsnorm OK: bit-exact match {ref}")
