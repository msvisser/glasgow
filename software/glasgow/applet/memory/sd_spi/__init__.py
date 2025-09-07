import contextlib
import logging
import struct
import sys
import time
from typing import Optional

from amaranth import *
from amaranth.lib import crc, enum, io, stream, wiring
from amaranth.lib.wiring import In, Out

from glasgow.abstract import AbstractAssembly, ClockDivisor, GlasgowPin
from glasgow.applet import GlasgowAppletError, GlasgowAppletV2
from glasgow.gateware import spi
from glasgow.support.bits import bits
from glasgow.support.bitstruct import bitstruct
from glasgow.support.logging import dump_hex

__all__ = ["SdSpiError", "SdSpiInterface"]

crc7 = staticmethod(crc.catalog.CRC7_MMC(data_width=8).compute)
crc16 = staticmethod(crc.catalog.CRC16_XMODEM(data_width=8).compute)


class SdSpiError(GlasgowAppletError):
    pass


class SdCardVersion(enum.Enum):
    SDV1 = enum.auto()
    SDV2 = enum.auto()
    SDHC = enum.auto()


SdCardSpecificData = bitstruct(
    "SdCardSpecificData",
    128,
    [
        (None, 1),
        ("checksum", 7),
        (None, 118),
        ("version", 2),
    ],
)

SdCardSpecificDataV1 = bitstruct(
    "SdCardSpecificDataV1",
    128,
    [
        (None, 1),
        ("checksum", 7),
        (None, 2),
        ("file_format", 2),
        ("temporary_write_protection", 1),
        ("permanent_write_protection", 1),
        ("copy_flag", 1),
        ("file_format_group", 1),
        (None, 5),
        ("partial_blocks_for_write_allowed", 1),
        ("max_write_data_block_length", 4),
        ("write_speed_factor", 3),
        (None, 2),
        ("write_protect_group_enable", 1),
        ("write_protect_group_size", 7),
        ("erase_sector_size", 7),
        ("erase_single_block_enable", 1),
        ("device_size_multiplier", 3),
        ("max_write_current_vdd_max", 3),
        ("max_write_current_vdd_min", 3),
        ("max_read_current_vdd_max", 3),
        ("max_read_current_vdd_min", 3),
        ("device_size", 12),
        (None, 2),
        ("dsr_implemented", 1),
        ("read_block_misalignment", 1),
        ("write_block_misalignment", 1),
        ("partial_blocks_for_read_allowed", 1),
        ("max_read_data_block_length", 4),
        ("card_command_class", 12),
        ("max_data_transfer_rate", 8),
        ("data_read_access_time_2", 8),
        ("data_read_access_time_1", 8),
        (None, 6),
        ("version", 2),
    ],
)

SdCardSpecificDataV2 = bitstruct(
    "SdCardSpecificDataV2",
    128,
    [
        (None, 1),
        ("checksum", 7),
        (None, 2),
        ("file_format", 2),
        ("temporary_write_protection", 1),
        ("permanent_write_protection", 1),
        ("copy_flag", 1),
        ("file_format_group", 1),
        (None, 5),
        ("partial_blocks_for_write_allowed", 1),
        ("max_write_data_block_length", 4),
        ("write_speed_factor", 3),
        (None, 2),
        ("write_protect_group_enable", 1),
        ("write_protect_group_size", 7),
        ("erase_sector_size", 7),
        ("erase_single_block_enable", 1),
        (None, 1),
        ("device_size", 22),
        (None, 6),
        ("dsr_implemented", 1),
        ("read_block_misalignment", 1),
        ("write_block_misalignment", 1),
        ("partial_blocks_for_read_allowed", 1),
        ("max_read_data_block_length", 4),
        ("card_command_class", 12),
        ("max_data_transfer_rate", 8),
        ("data_read_access_time_2", 8),
        ("data_read_access_time_1", 8),
        (None, 6),
        ("version", 2),
    ],
)


SdCardIdentification = bitstruct(
    "SdCardIdentification",
    128,
    [
        (None, 1),
        ("checksum", 7),
        ("manufacturing_month", 4),
        ("manufacturing_year", 8),
        (None, 4),
        ("product_serial_number", 32),
        ("product_revision_minor", 4),
        ("product_revision_major", 4),
        ("product_name", 40),
        ("oem_application_id", 16),
        ("manufacturer_id", 8),
    ],
)


class SdSpiCommand(enum.Enum, shape=4):
    Select = 0
    Transfer = 1
    Sync = 2
    WakeUp = 3
    Command = 4
    ReadBlocks = 5


class SdBlockSize(enum.Enum, shape=1):
    _16B = 0
    _512B = 1


class SdSpiControllerComponent(wiring.Component):
    i_stream: In(stream.Signature(8))
    o_stream: Out(stream.Signature(8))

    divisor: In(16)

    def __init__(self, ports, *, offset=None, us_cycles):
        self._ports = ports
        self._offset = offset
        self._us_cycles = us_cycles

        super().__init__()

    def elaborate(self, platform):
        m = Module()

        if self._ports.copi is None:
            self._ports.copi = io.SimulationPort("o", 1)
        if self._ports.cipo is None:
            self._ports.cipo = io.SimulationPort("i", 1)

        m.submodules.ctrl = ctrl = spi.Controller(
            self._ports,
            # Offset sampling by ~10 ns to compensate for 10..15 ns of roundtrip delay caused by
            # the level shifters (5 ns each) and FPGA clock-to-out (5 ns).
            offset=1 if self._offset is None else self._offset,
            chip_count=len(self._ports.cs) + 1,
        )
        m.d.comb += ctrl.divisor.eq(self.divisor)

        dummy_cs = len(self._ports.cs) + 1

        command = Signal(SdSpiCommand)
        chip = Signal(range(1 + len(self._ports.cs)))
        mode = Signal(spi.Mode)
        is_put = mode.as_value().matches(spi.Mode.Put, spi.Mode.Swap)
        is_get = mode.as_value().matches(spi.Mode.Get, spi.Mode.Swap)  # FIXME: amaranth-lang/amaranth#1462
        o_count = Signal(16)
        i_count = Signal(16)

        should_tx = Signal()
        pending_tx = Signal()
        pending_rx = Signal()
        command_response_length = Signal(4)
        command_response_skip_one = Signal()
        success = Signal()

        wait_for_ready_count = Signal(16)
        block_size = Signal(SdBlockSize)
        number_of_blocks = Signal(16)

        with m.FSM():
            with m.State("Read-Command"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    m.d.sync += command.eq(self.i_stream.payload[4:])
                    with m.Switch(self.i_stream.payload[4:]):
                        with m.Case(SdSpiCommand.Select):
                            m.d.sync += chip.eq(self.i_stream.payload[:4])
                            m.next = "Read-Command"
                        with m.Case(SdSpiCommand.Transfer):
                            m.d.sync += mode.eq(self.i_stream.payload[:4])
                            m.next = "Read-Count-0:8"
                        with m.Case(SdSpiCommand.Sync):
                            m.next = "Sync"
                        with m.Case(SdSpiCommand.WakeUp):
                            m.next = "Read-Count-0:8"
                        with m.Case(SdSpiCommand.Command):
                            m.d.sync += should_tx.eq(1)
                            m.d.sync += wait_for_ready_count.eq(8)
                            m.d.sync += o_count.eq(6)
                            m.d.sync += command_response_length.eq(self.i_stream.payload[:3])
                            m.d.sync += command_response_skip_one.eq(self.i_stream.payload[3])
                            m.next = "Command-Wait-For-Ready"
                        with m.Case(SdSpiCommand.ReadBlocks):
                            m.d.sync += block_size.eq(self.i_stream.payload[0])
                            m.next = "Read-Count-0:8"

            with m.State("Read-Count-0:8"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    m.d.sync += o_count[0:8].eq(self.i_stream.payload)
                    m.d.sync += i_count[0:8].eq(self.i_stream.payload)
                    m.next = "Read-Count-8:16"

            with m.State("Read-Count-8:16"):
                m.d.comb += self.i_stream.ready.eq(1)
                with m.If(self.i_stream.valid):
                    m.d.sync += o_count[8:16].eq(self.i_stream.payload)
                    m.d.sync += i_count[8:16].eq(self.i_stream.payload)
                    with m.Switch(command):
                        with m.Case(SdSpiCommand.Transfer):
                            m.next = "Transfer"
                        with m.Case(SdSpiCommand.WakeUp):
                            m.next = "WakeUp"
                        with m.Case(SdSpiCommand.ReadBlocks):
                            m.d.sync += number_of_blocks.eq(Cat(i_count[0:8], self.i_stream.payload))
                            m.d.sync += i_count.eq(Mux(block_size == SdBlockSize._512B, 514, 18))
                            m.d.sync += o_count.eq(Mux(block_size == SdBlockSize._512B, 514, 18))
                            m.d.sync += should_tx.eq(1)
                            m.d.sync += wait_for_ready_count.eq(10000)
                            m.next = "Data-Block-Wait-For-Ready"

            with m.State("Transfer"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(mode),
                    ctrl.i_stream.p.data.eq(self.i_stream.payload),
                    self.o_stream.payload.eq(ctrl.o_stream.p.data),
                ]
                with m.If(o_count != 0):
                    with m.If(is_put):
                        m.d.comb += ctrl.i_stream.valid.eq(self.i_stream.valid)
                        m.d.comb += self.i_stream.ready.eq(ctrl.i_stream.ready)
                    with m.Else():
                        m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += o_count.eq(o_count - 1)
                with m.If(i_count != 0):
                    with m.If(is_get):
                        m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                        m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                        with m.If(ctrl.o_stream.valid & ctrl.o_stream.ready):
                            m.d.sync += i_count.eq(i_count - 1)
                with m.If((o_count == 0) & ((i_count == 0) | ~is_get)):
                    m.next = "Read-Command"

            with m.State("Sync"):
                m.d.comb += self.o_stream.valid.eq(1)
                with m.If(self.o_stream.ready):
                    m.next = "Read-Command"

            with m.State("WakeUp"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(dummy_cs),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Dummy),
                    ctrl.i_stream.p.data.eq(0xFF),
                ]

                with m.If(o_count != 0):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += o_count.eq(o_count - 1)
                with m.If(o_count == 0):
                    m.next = "Read-Command"

            with m.State("Command-Wait-For-Ready"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                ]

                with m.If(should_tx):
                    m.d.sync += should_tx.eq(0)
                    m.d.sync += pending_tx.eq(1)
                    m.d.sync += pending_rx.eq(1)
                with m.Elif(~pending_tx & ~pending_rx):
                    with m.If(wait_for_ready_count == 0):
                        m.next = "Command-Transmit"
                    with m.Else():
                        m.d.sync += should_tx.eq(1)

                with m.If(pending_tx):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += pending_tx.eq(0)

                with m.If(pending_rx):
                    m.d.comb += ctrl.o_stream.ready.eq(1)
                    with m.If(ctrl.o_stream.valid & ctrl.o_stream.ready):
                        m.d.sync += pending_rx.eq(0)
                        m.d.sync += wait_for_ready_count.eq(wait_for_ready_count - 1)
                        with m.If(ctrl.o_stream.p.data == 0xFF):
                            m.d.sync += wait_for_ready_count.eq(0)

            with m.State("Command-Transmit"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Put),
                    ctrl.i_stream.p.data.eq(self.i_stream.payload),
                ]

                with m.If(o_count != 0):
                    m.d.comb += ctrl.i_stream.valid.eq(self.i_stream.valid)
                    m.d.comb += self.i_stream.ready.eq(ctrl.i_stream.ready)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += o_count.eq(o_count - 1)
                with m.If(o_count == 0):
                    m.d.sync += should_tx.eq(1)
                    with m.If(command_response_skip_one):
                        m.d.sync += wait_for_ready_count.eq(1)
                        m.next = "Command-Receive-R1-Skip"
                    with m.Else():
                        m.d.sync += wait_for_ready_count.eq(8)
                        m.next = "Command-Receive-R1"

            with m.State("Command-Receive-R1-Skip"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                ]

                with m.If(should_tx):
                    m.d.sync += should_tx.eq(0)
                    m.d.sync += pending_tx.eq(1)
                    m.d.sync += pending_rx.eq(1)
                with m.Elif(~pending_tx & ~pending_rx):
                    with m.If(wait_for_ready_count == 0):
                        m.d.sync += wait_for_ready_count.eq(7)
                        m.next = "Command-Receive-R1"
                    with m.Else():
                        m.d.sync += should_tx.eq(1)

                with m.If(pending_tx):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += pending_tx.eq(0)

                with m.If(pending_rx):
                    m.d.comb += ctrl.o_stream.ready.eq(1)
                    with m.If(ctrl.o_stream.valid):
                        m.d.sync += pending_rx.eq(0)
                        m.d.sync += wait_for_ready_count.eq(wait_for_ready_count - 1)

            with m.State("Command-Receive-R1"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                    self.o_stream.payload.eq(ctrl.o_stream.p.data),
                ]

                with m.If(should_tx):
                    m.d.sync += should_tx.eq(0)
                    m.d.sync += pending_tx.eq(1)
                    m.d.sync += pending_rx.eq(1)
                with m.Elif(~pending_tx & ~pending_rx):
                    with m.If(success):
                        m.d.sync += success.eq(0)
                        with m.If(command_response_length > 0):
                            m.d.sync += o_count.eq(command_response_length)
                            m.d.sync += i_count.eq(command_response_length)
                            m.next = "Command-Receive-Data"
                        with m.Else():
                            m.next = "Read-Command"
                    with m.Elif(wait_for_ready_count == 0):
                        m.d.comb += self.o_stream.valid.eq(1)
                        m.d.comb += self.o_stream.payload.eq(0xFF)
                        with m.If(self.o_stream.ready):
                            m.next = "Read-Command"
                    with m.Else():
                        m.d.sync += should_tx.eq(1)

                with m.If(pending_tx):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += pending_tx.eq(0)

                with m.If(pending_rx):
                    with m.If(ctrl.o_stream.valid):
                        with m.If(ctrl.o_stream.ready):
                            m.d.sync += pending_rx.eq(0)
                            m.d.sync += wait_for_ready_count.eq(wait_for_ready_count - 1)

                        with m.If((ctrl.o_stream.p.data & 0x80) == 0):
                            m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                            m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                            with m.If(ctrl.o_stream.ready):
                                m.d.sync += success.eq(1)
                        with m.Else():
                            m.d.comb += ctrl.o_stream.ready.eq(1)

            with m.State("Command-Receive-Data"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                    self.o_stream.payload.eq(ctrl.o_stream.p.data),
                ]

                with m.If(o_count != 0):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += o_count.eq(o_count - 1)

                with m.If(i_count != 0):
                    m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                    m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                    with m.If(ctrl.o_stream.valid & ctrl.o_stream.ready):
                        m.d.sync += i_count.eq(i_count - 1)

                with m.If((o_count == 0) & (i_count == 0)):
                    m.next = "Read-Command"

            with m.State("Data-Block-Wait-For-Ready"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                    self.o_stream.payload.eq(ctrl.o_stream.p.data),
                ]

                with m.If(should_tx):
                    m.d.sync += should_tx.eq(0)
                    m.d.sync += pending_tx.eq(1)
                    m.d.sync += pending_rx.eq(1)
                with m.Elif(~pending_tx & ~pending_rx):
                    with m.If(success):
                        m.d.sync += success.eq(0)
                        m.next = "Data-Block-Receive-Data"
                    with m.Elif(wait_for_ready_count == 0):
                        m.next = "Read-Command"
                    with m.Else():
                        m.d.sync += should_tx.eq(1)

                with m.If(pending_tx):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += pending_tx.eq(0)

                with m.If(pending_rx):
                    with m.If(ctrl.o_stream.valid):
                        with m.If(ctrl.o_stream.ready):
                            m.d.sync += pending_rx.eq(0)
                            m.d.sync += wait_for_ready_count.eq(wait_for_ready_count - 1)

                        with m.If(ctrl.o_stream.p.data == 0xFE):
                            m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                            m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                            with m.If(ctrl.o_stream.ready):
                                m.d.sync += success.eq(1)
                        with m.Elif((ctrl.o_stream.p.data[4:] == 0) & (ctrl.o_stream.p.data[:4] != 0)):
                            m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                            m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                            with m.If(ctrl.o_stream.ready):
                                m.d.sync += wait_for_ready_count.eq(0)
                        with m.Elif(wait_for_ready_count == 1):
                            m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                            m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                        with m.Else():
                            m.d.comb += ctrl.o_stream.ready.eq(1)

            with m.State("Data-Block-Receive-Data"):
                m.d.comb += [
                    ctrl.i_stream.p.chip.eq(chip),
                    ctrl.i_stream.p.mode.eq(spi.Mode.Swap),
                    ctrl.i_stream.p.data.eq(0xFF),
                    self.o_stream.payload.eq(ctrl.o_stream.p.data),
                ]

                with m.If(o_count != 0):
                    m.d.comb += ctrl.i_stream.valid.eq(1)
                    with m.If(ctrl.i_stream.valid & ctrl.i_stream.ready):
                        m.d.sync += o_count.eq(o_count - 1)

                with m.If(i_count != 0):
                    m.d.comb += self.o_stream.valid.eq(ctrl.o_stream.valid)
                    m.d.comb += ctrl.o_stream.ready.eq(self.o_stream.ready)
                    with m.If(ctrl.o_stream.valid & ctrl.o_stream.ready):
                        m.d.sync += i_count.eq(i_count - 1)

                with m.If((o_count == 0) & (i_count == 0)):
                    m.d.sync += number_of_blocks.eq(number_of_blocks - 1)
                    with m.If(number_of_blocks > 1):
                        m.d.sync += i_count.eq(Mux(block_size == SdBlockSize._512B, 514, 18))
                        m.d.sync += o_count.eq(Mux(block_size == SdBlockSize._512B, 514, 18))
                        m.d.sync += should_tx.eq(1)
                        m.d.sync += wait_for_ready_count.eq(10000)
                        m.next = "Data-Block-Wait-For-Ready"
                    with m.Else():
                        m.next = "Read-Command"

        return m


class SdSpiInterface:
    def __init__(
        self,
        logger: logging.Logger,
        assembly: AbstractAssembly,
        *,
        cs: GlasgowPin,
        sck: GlasgowPin,
        copi: Optional[GlasgowPin] = None,
        cipo: Optional[GlasgowPin] = None,
    ):
        self._logger = logger
        self._level = logging.DEBUG if self._logger.name == __name__ else logging.TRACE

        ports = assembly.add_port_group(cs=cs, sck=sck, copi=copi, cipo=cipo)
        component = assembly.add_submodule(SdSpiControllerComponent(ports, us_cycles=int(1 / (assembly.sys_clk_period * 1_000_000))))
        self._pipe = assembly.add_inout_pipe(component.o_stream, component.i_stream)
        self._clock = assembly.add_clock_divisor(component.divisor, ref_period=assembly.sys_clk_period, name="sck")

        self._active = None

    def _log(self, message, *args):
        self._logger.log(self._level, message, *args)

    def _log_trace(self, message, *args):
        self._logger.log(logging.TRACE, message, *args)

    @property
    def clock(self) -> ClockDivisor:
        return self._clock

    @staticmethod
    def _chunked(items, *, count=0xffff):
        while items:
            yield items[:count]
            items = items[count:]

    @contextlib.asynccontextmanager
    async def _select(self, index=0):
        assert self._active is None, "chip already selected"
        assert index in range(8)
        try:
            self._log_trace("select chip=%d", index)
            await self._pipe.send(struct.pack("<B", (SdSpiCommand.Select.value << 4) | (1 + index)))
            self._active = index
            yield
        finally:
            self._log_trace("deselect")
            await self._pipe.send(
                struct.pack(
                    "<BBH",
                    (SdSpiCommand.Select.value << 4) | 0,
                    (SdSpiCommand.Transfer.value << 4) | spi.Mode.Dummy.value,
                    1,
                )
            )
            await self._pipe.flush()
            self._active = None

    async def _synchronize(self):
        self._log_trace("sync-o")
        await self._pipe.send(struct.pack("<B", (SdSpiCommand.Sync.value << 4)))
        await self._pipe.flush()
        await self._pipe.recv(1)
        self._log_trace("sync-i")

    async def _wakeup(self, cycles=74):
        """Wake up the SD card by sending 74 clock cycles."""
        self._log_trace("wakeup cycles=%d", cycles)
        await self._pipe.send(struct.pack("<BH", (SdSpiCommand.WakeUp.value << 4), cycles))
        await self._synchronize()

    async def _command_internal(self, command, argument=0, response_length=0, skip_first_response_byte=False) -> tuple[int, memoryview]:
        """Send a command without selecting a chip."""
        self._log_trace("command_internal: cmd=%d arg=%08X rsp_len=%d", command, argument, response_length)
        assert self._active is not None, "no chip selected"
        assert response_length <= 7, "invalid response length"

        command_bytes = bytes(
            [
                0x40 | (command & 0x3F),
                (argument >> 24) & 0xFF,
                (argument >> 16) & 0xFF,
                (argument >> 8) & 0xFF,
                argument & 0xFF,
            ]
        )
        command_crc = (crc7(command_bytes) << 1) | 1

        await self._pipe.send(
            struct.pack(
                "<B5sB",
                (SdSpiCommand.Command.value << 4) | ((1 << 3) if skip_first_response_byte else 0) | response_length,
                command_bytes,
                command_crc,
            )
        )
        await self._pipe.flush()

        response_r1 = (await self._pipe.recv(1))[0]
        if (response_r1 & 0x80) == 0 and response_length > 0:
            response_data = await self._pipe.recv(response_length)
            return response_r1, bytes(response_data)
        else:
            return response_r1, None

    async def _read_data_blocks(self, data_block_length, number_of_blocks, callback=None, ignore_crc=False) -> tuple[int, bytes]:
        """Read a data block without selecting a chip."""
        assert self._active is not None, "no chip selected"
        assert data_block_length in [16, 512]
        self._log_trace("read_data_block: blk_len=%d", data_block_length)

        last_update = 0
        output_data = bytearray()

        for chunk in self._chunked(range(number_of_blocks)):
            await self._pipe.send(
                struct.pack(
                    "<BH",
                    (SdSpiCommand.ReadBlocks.value << 4) | (SdBlockSize._512B.value if data_block_length == 512 else SdBlockSize._16B.value),
                    len(chunk),
                )
            )
            await self._pipe.flush()

            for _ in chunk:
                data_start_token = (await self._pipe.recv(1))[0]
                self._log_trace("data start token: %02X", data_start_token)
                if data_start_token != 0xFE:
                    return data_start_token, bytes(output_data)

                block_data_and_crc = await self._pipe.recv(data_block_length + 2)
                self._log_trace("block data and crc: %s", block_data_and_crc.hex())

                block_data = block_data_and_crc[:-2]
                block_crc = (block_data_and_crc[-2] << 8) | block_data_and_crc[-1]
                calculated_crc = crc16(block_data)
                if block_crc != calculated_crc and not ignore_crc:
                    raise SdSpiError(f"CRC mismatch: expected {calculated_crc} got {block_crc}")

                output_data.extend(block_data)

                if callback is not None and len(output_data) - last_update > 0x10000:
                    callback(len(output_data), number_of_blocks * data_block_length)

        return data_start_token, bytes(output_data)

    async def _command(self, command, argument=0, response_length=0) -> tuple[int, memoryview | None]:
        self._log_trace("command: cmd=%d arg=%08X rsp_len=%d", command, argument, response_length)

        async with self._select():
            response_r1, response_data = await self._command_internal(command, argument, response_length)

        return response_r1, response_data

    async def _app_command(self, command, argument=0, response_length=0) -> tuple[int, memoryview | None]:
        self._log_trace("app_command: cmd=%d arg=%08X rsp_len=%d", command, argument, response_length)

        status, _ = await self._command(55)
        if status > 1:
            raise SdSpiError(f"App command prefix failed with status {status}")

        return await self._command(command, argument, response_length)

    async def _command_with_data_block(self, command, argument=0, response_length=0, data_block_length=512, ignore_crc=False) -> tuple[int, memoryview | None, int, bytes | None]:
        self._log_trace("command_with_data_block: cmd=%d arg=%08X rsp_len=%d data_len=%d", command, argument, response_length, data_block_length)

        async with self._select():
            response_r1, response_data = await self._command_internal(command, argument, response_length)
            if response_r1 > 1:
                return response_r1, None, None, None

            data_start_token, block_data = await self._read_data_blocks(data_block_length, number_of_blocks=1, ignore_crc=ignore_crc)

        return response_r1, response_data, data_start_token, block_data

    async def initialize(self) -> SdCardVersion:
        """Initialize the SD card."""
        self._log("Initializing SD card")
        await self._wakeup()

        # CMD0: GO_IDLE_STATE
        status, _ = await self._command(0)
        if status != 1:
            raise SdSpiError(f"CMD0 failed with status {status}")
        self._log("CMD0: GO_IDLE_STATE -> OK")

        # CMD8: SEND_IF_COND
        status, response = await self._command(8, argument=0x000001AA, response_length=4)
        if status == 5:
            self._log("CMD8: SEND_IF_COND -> Illegal command (SD v1.x or MMC)")
            sd_version = 1
        elif status == 1:
            if response != bytes([0x00, 0x00, 0x01, 0xAA]):
                raise SdSpiError(f"CMD8: SEND_IF_COND -> Invalid response {response.hex()}")
            self._log("CMD8: SEND_IF_COND -> OK (SD v2.x)")
            sd_version = 2
        else:
            raise SdSpiError(f"CMD8: SEND_IF_COND failed with status {status}")

        # ACMD41: SD_SEND_OP_COND (with HCS bit for SD v2.x)
        argument = 0x40000000 if sd_version == 2 else 0x00000000
        for _ in range(100):
            status, _ = await self._app_command(41, argument=argument)
            if status == 0:
                break
        else:
            raise SdSpiError("ACMD41: SD_SEND_OP_COND -> Timeout waiting for card to be ready")
        self._log("ACMD41: SD_SEND_OP_COND -> OK")

        # CMD58: READ_OCR
        status, ocr = await self._command(58, response_length=4)
        if status != 0:
            raise SdSpiError(f"CMD58: READ_OCR failed with status {status}")
        self._log("CMD58: READ_OCR -> OK, OCR=%s", dump_hex(ocr))

        # TODO: Check with CMD6 if high speed mode is supported before enabling it
        # CMD6: SWITCH_FUNC
        status, _, data_start_token, data = await self._command_with_data_block(6, argument=0x80FFFFF1, data_block_length=512, ignore_crc=True)
        if status != 0:
            raise SdSpiCommand(f"CMD6: SWITCH_FUNC failed with status {status}")
        if data_start_token != 0xFE:
            raise SdSpiCommand(f"CMD6: SWITCH_FUNC failed with data start token {data_start_token}")
        self._log("CMD6: SWITCH_FUNC -> OK, data=%s", dump_hex(data))

        if sd_version == 2 and (ocr[0] & 0x40):
            return SdCardVersion.SDHC
        elif sd_version == 2:
            return SdCardVersion.SDV2
        else:
            return SdCardVersion.SDV1

    async def read_card_specific_data(self):
        """Read the card specific data (CSD) from the SD card."""
        status, _, data_start_token, data = await self._command_with_data_block(9, data_block_length=16)
        if status != 0:
            raise SdSpiError(f"CMD9: SEND_CSD failed with status {status}")
        if data_start_token != 0xFE:
            raise SdSpiError(f"CMD9: SEND_CSD failed with data start token {data_start_token}")

        data_rev = bytes(reversed(data))
        calculated_checksum = crc7(data[:15])
        csd = SdCardSpecificData.from_bytes(data_rev)
        if csd.checksum != calculated_checksum:
            raise SdSpiError("Invalid checksum for SdCardSpecificData")

        if csd.version == 0:
            csd = SdCardSpecificDataV1.from_bytes(data_rev)
        elif csd.version == 1:
            csd = SdCardSpecificDataV2.from_bytes(data_rev)
        else:
            raise SdSpiError("Unknown CSD version")

        self._log("CSD: %s", dump_hex(data))

        return csd

    async def read_card_identification(self):
        """Read the card identification (CID) from the SD card."""
        status, _, data_start_token, data = await self._command_with_data_block(10, data_block_length=16)
        if status != 0:
            raise SdSpiError(f"CMD10: SEND_CID failed with status {status}")
        if data_start_token != 0xFE:
            raise SdSpiError(f"CMD10: SEND_CID failed with data start token {data_start_token}")

        data_rev = bytes(reversed(data))
        calculated_checksum = crc7(data[:15])
        cid = SdCardIdentification.from_bytes(data_rev)
        if cid.checksum != calculated_checksum:
            raise SdSpiError("Invalid checksum for SdCardIdentification")

        self._log("CID: %s", dump_hex(data))

        return cid

    async def read_block(self, address: int) -> bytes:
        """Read a 512-byte block from the SD card at the specified address."""
        self._log(f"Reading block at address {address:#x}")
        status, _, data_start_token, data = await self._command_with_data_block(17, argument=address, data_block_length=512)

        if status != 0:
            raise SdSpiError(f"CMD17: READ_SINGLE_BLOCK failed with status {status}")
        if data_start_token != 0xFE:
            raise SdSpiError(f"CMD17: READ_SINGLE_BLOCK failed with data start token {data_start_token}")

        self._log(f"Read block at address {address:#x} -> OK")
        return data

    async def read_blocks(self, start_block: int, count: int, callback=None) -> bytes:
        """Read multiple 512-byte blocks from the SD card starting at the specified address."""
        self._log(f"Reading {count} blocks starting at address {start_block:#x}")

        async with self._select():
            status, _ = await self._command_internal(18, start_block)
            if status != 0:
                raise SdSpiError(f"CMD18: READ_MULTIPLE_BLOCK failed with status {status}")

            data_start_token, total_data = await self._read_data_blocks(512, count, callback=callback)
            if data_start_token != 0xFE:
                raise SdSpiError(f"CMD18: READ_MULTIPLE_BLOCK failed with data start token {data_start_token}")

            status, _ = await self._command_internal(12, skip_first_response_byte=True)
            if status != 0:
                raise SdSpiError(f"CMD12: STOP_TRANSMISSION failed with status {status}")

        return bytes(total_data)


class MemorySdSpiApplet(GlasgowAppletV2):
    logger = logging.getLogger(__name__)
    help = "read and write SD cards using SPI"
    description = """
    Identify, read, write, and erase SD cards using SPI.
    """

    @classmethod
    def add_build_arguments(cls, parser, access):
        access.add_voltage_argument(parser)
        access.add_pins_argument(parser, "cs", default=True, required=True)
        access.add_pins_argument(parser, "sck", default=True, required=True)
        access.add_pins_argument(parser, "copi", default=True, required=True)
        access.add_pins_argument(parser, "cipo", default=True, required=True)

    def build(self, args):
        with self.assembly.add_applet(self):
            self.assembly.use_voltage(args.voltage)
            self.sd_spi_iface = SdSpiInterface(
                self.logger,
                self.assembly,
                cs=args.cs,
                sck=args.sck,
                copi=args.copi,
                cipo=args.cipo,
            )

    @classmethod
    def add_setup_arguments(cls, parser):
        parser.add_argument("-f", "--frequency", metavar="FREQ", type=int, default=500, help="set start-up SCK frequency to FREQ kHz (default: %(default)s)")
        parser.add_argument("-x", "--high-frequency", metavar="FREQ", type=int, default=4800, help="set high SCK frequency to FREQ kHz (default: %(default)s)")

    async def setup(self, args):
        await self.sd_spi_iface.clock.set_frequency(args.frequency * 1000)
        self.high_frequency = args.high_frequency * 1000

    @classmethod
    def add_run_arguments(cls, parser):
        p_operation = parser.add_subparsers(dest="operation", metavar="OPERATION", required=True)

        p_identify = p_operation.add_parser("identify", help="identify the SD card")

        p_read = p_operation.add_parser("read", help="read data from the SD card")
        p_read.add_argument("block_address", type=lambda x: int(x, 0), help="block address to read from")

        p_read_multiple = p_operation.add_parser("read-multiple", help="read multiple blocks from the SD card")
        p_read_multiple.add_argument("block_address", type=lambda x: int(x, 0), help="block address to read from")
        p_read_multiple.add_argument("count", type=lambda x: int(x, 0), help="number of blocks to read")

    async def run(self, args):
        try:
            sd_card_version = await self.sd_spi_iface.initialize()
            await self.sd_spi_iface.clock.set_frequency(self.high_frequency)

            if args.operation == "identify":
                card_identification = await self.sd_spi_iface.read_card_identification()
                card_specific_data = await self.sd_spi_iface.read_card_specific_data()

                match sd_card_version:
                    case SdCardVersion.SDV1:
                        self.logger.info("SD Card Version: SD v1.x (SDSC)")
                    case SdCardVersion.SDV2:
                        self.logger.info("SD Card Version: SD v2.x (SDSC)")
                    case SdCardVersion.SDHC:
                        self.logger.info("SD Card Version: SDHC/SDXC")

                self.logger.info("")
                self.logger.info("SD Card Identification (CID):")
                self.logger.info(f"    Manufacturer ID: 0x{card_identification.manufacturer_id:02x}")
                self.logger.info(f"    OEM/Application ID: '{bytes(reversed(bits(card_identification.oem_application_id).to_bytes())).decode('ascii')}'")
                self.logger.info(f"    Product name: '{bytes(reversed(bits(card_identification.product_name).to_bytes())).decode('ascii')}'")
                self.logger.info(f"    Product revision: {card_identification.product_revision_major}.{card_identification.product_revision_minor}")
                self.logger.info(f"    Product serial number: {card_identification.product_serial_number}")
                self.logger.info(f"    Manufacturing date: {card_identification.manufacturing_month:02}-{2000 + card_identification.manufacturing_year:04}")

                self.logger.info("")
                self.logger.info("SD Card Specific Data (CSD):")

                if card_specific_data.version == 0:
                    self.logger.info(f"    Data read access time: {card_specific_data.data_read_access_time_1:07b}")
                    self.logger.info(f"    Data read access time in CLK cycles: {100 * card_specific_data.data_read_access_time_2}")

                if card_specific_data.max_data_transfer_rate == 0b0_0110_010:
                    transfer_rate = "25MHz (max 12.5MB/s in SD 4-bit mode)"
                elif card_specific_data.max_data_transfer_rate == 0b0_1011_010:
                    transfer_rate = "50MHz (max 25.0MB/s in SD 4-bit mode)"
                else:
                    transfer_rate = "Unknown"

                self.logger.info(f"    Max data transfer rate: {transfer_rate}")
                self.logger.info(f"    Card command class: {card_specific_data.card_command_class:012b}")

                if card_specific_data.version == 0:
                    self.logger.info(f"    Max read data block length: {2 ** card_specific_data.max_read_data_block_length}")
                    self.logger.info(f"    Partial blocks for read allowed: {card_specific_data.partial_blocks_for_read_allowed}")
                    self.logger.info(f"    Write block misalignment: {card_specific_data.write_block_misalignment}")
                    self.logger.info(f"    Read block misalignment: {card_specific_data.read_block_misalignment}")

                self.logger.info(f"    DSR implemented: {card_specific_data.dsr_implemented}")

                if card_specific_data.version == 0:
                    self.logger.info(f"    Device size: {card_specific_data.device_size}")
                    self.logger.info(f"    Max read current @ VDD max: {card_specific_data.max_read_current_vdd_min}")
                    self.logger.info(f"    Max read current @ VDD max: {card_specific_data.max_read_current_vdd_max}")
                    self.logger.info(f"    Max write current @ VDD min: {card_specific_data.max_write_current_vdd_min}")
                    self.logger.info(f"    Max write current @ VDD max: {card_specific_data.max_write_current_vdd_max}")
                    self.logger.info(f"    Device size multiplier: {card_specific_data.device_size_multiplier}")
                elif card_specific_data.version == 1:
                    self.logger.info(f"    Device size: {512 * 1024 * (card_specific_data.device_size + 1)} B ({1024 * (card_specific_data.device_size + 1)} blocks)")

                if card_specific_data.version == 0:
                    self.logger.info(f"    Erase single block enabled: {card_specific_data.erase_single_block_enable}")
                    self.logger.info(f"    Erase sector size: {card_specific_data.erase_sector_size}")
                    self.logger.info(f"    Write protect group size: {card_specific_data.write_protect_group_size}")
                    self.logger.info(f"    Write protect group enable: {card_specific_data.write_protect_group_enable}")
                    self.logger.info(f"    Write speed factor: {card_specific_data.write_speed_factor}")
                    self.logger.info(f"    Max write data block length: {2 ** card_specific_data.max_write_data_block_length}")
                    self.logger.info(f"    Partial blocks for write allowed: {card_specific_data.partial_blocks_for_write_allowed}")
                    self.logger.info(f"    File format group: {card_specific_data.file_format_group}")

                self.logger.info(f"    Copy flag: {card_specific_data.copy_flag}")
                self.logger.info(f"    Permanent write protection: {card_specific_data.permanent_write_protection}")
                self.logger.info(f"    Temporary write protection: {card_specific_data.temporary_write_protection}")

                if card_specific_data.version == 0:
                    self.logger.info(f"    File format: {card_specific_data.file_format}")

            elif args.operation == "read":
                self.logger.info(f"Reading single block from address {args.block_address:#x}")
                data = await self.sd_spi_iface.read_block(args.block_address)
                print(data.hex())
                with open(f"sd_block_{args.block_address:08x}.bin", "wb") as f:
                    f.write(data)

            elif args.operation == "read-multiple":
                self.logger.info(f"Reading {args.count} blocks starting at address {args.block_address:#x}")

                start_time = time.time()
                data = await self.sd_spi_iface.read_blocks(args.block_address, args.count, self._show_progress)
                duration = time.time() - start_time
                self._show_progress(0, 0)

                bytes_per_second = len(data) / duration
                max_bytes_per_second = self.high_frequency / 8
                self.logger.info(f"Read {len(data)} bytes in {duration:.3f} seconds, {bytes_per_second / 1000:.2f} KB/s, usage {100 * bytes_per_second / max_bytes_per_second:.1f}%")

                with open(f"sd_blocks_{args.block_address:08x}_{args.count}.bin", "wb") as f:
                    f.write(data)

        except SdSpiError as e:
            self.logger.error("Error: %s", e)
        finally:
            await self.device.set_voltage("AB", 0.0)

    @staticmethod
    def _show_progress(done, total):
        if sys.stdout.isatty():
            sys.stdout.write("\r\033[0K")
            if done < total:
                sys.stdout.write(f"{done}/{total} bytes done ({100*done/total:.2f}%)")
            sys.stdout.flush()

    @classmethod
    def tests(cls):
        from . import test

        return test.MemorySdSpiAppletTestCase
