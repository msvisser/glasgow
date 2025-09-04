from amaranth import *
from amaranth.lib import io

from glasgow.gateware.ports import PortGroup
from glasgow.simulation.assembly import SimulationAssembly
from glasgow.applet import (
    GlasgowAppletV2TestCase,
    synthesis_test,
    applet_v2_simulation_test,
)
from . import MemorySdSpiApplet


class MemorySdSpiAppletTestCase(GlasgowAppletV2TestCase, applet=MemorySdSpiApplet):
    # @synthesis_test
    # def test_build(self):
    #     self.assertBuilds()

    simulation_args = ["-f", "50"]

    def prepare_target(self, assembly: SimulationAssembly):
        ctl_ports = PortGroup(
            cs=assembly.get_pin("A0"),
            sck=assembly.get_pin("A1"),
            copi=assembly.get_pin("A2"),
            cipo=assembly.get_pin("A3"),
        )
        tgt_ports = PortGroup(
            cs=io.SimulationPort("i", 1),
            sck=io.SimulationPort("i", 1),
            copi=io.SimulationPort("i", 1),
            cipo=io.SimulationPort("o", 1),
        )

        m = Module()
        m.d.comb += [
            tgt_ports.cs.i.eq(ctl_ports.cs.o),
            tgt_ports.sck.i.eq(ctl_ports.sck.o),
            tgt_ports.copi.i.eq(ctl_ports.copi.o),
            ctl_ports.cipo.i.eq(tgt_ports.cipo.o),
        ]

        self.shreg_in = 0
        self.shreg_in_cnt = 0
        self.last_sck = 0
        self.response_bytes = []
        self.shreg_out = 0
        self.shreg_out_cnt = 0

        async def testbench(ctx):
            ctx.set(tgt_ports.cipo.o, 1)
            self.last_sck = ctx.get(tgt_ports.sck.i)
            async for _ in ctx.tick():
                cs = ctx.get(tgt_ports.cs.i)
                sck = ctx.get(tgt_ports.sck.i)

                if not cs:
                    if self.last_sck and not sck:
                        if self.shreg_out_cnt == 0:
                            if len(self.response_bytes) > 0:
                                self.shreg_out = self.response_bytes[0]
                                del self.response_bytes[0]
                            else:
                                self.shreg_out = 0xFF
                            self.shreg_out_cnt = 8

                        ctx.set(tgt_ports.cipo.o, self.shreg_out >> 7)
                        self.shreg_out <<= 1
                        self.shreg_out_cnt -= 1

                    if not self.last_sck and sck:
                        copi = ctx.get(tgt_ports.copi.i)
                        self.shreg_in = (self.shreg_in << 1) | copi
                        self.shreg_in_cnt += 1
                        if self.shreg_in_cnt == 8:
                            print(hex(self.shreg_in))
                            self.shreg_in = 0
                            self.shreg_in_cnt = 0

                self.last_sck = sck

        assembly.add_submodule(m)
        assembly.add_testbench(testbench, background=True)

    @applet_v2_simulation_test(prepare=prepare_target, args=simulation_args)
    async def test_wakeup(self, applet: MemorySdSpiApplet, ctx):
        self.response_bytes = [
            0x00, 0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request CMD0
            0xFF, 0x01, # Command response
            0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request CMD8
            0xFF, 0x01, 0x00, 0x00, 0x01, 0xAA, # Command response
            0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request CMD55
            0xFF, 0x01, # Command response
            0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request ACMD41
            0xFF, 0x00, # Command response
            0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request CMD58
            0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00, # Command response
        ]
        await applet.sd_spi_iface.initialize()

    @applet_v2_simulation_test(prepare=prepare_target, args=simulation_args)
    async def test_wakeup2(self, applet: MemorySdSpiApplet, ctx):
        self.response_bytes = [
            0x00, 0xFF, # Ready check
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, # Command request
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF # Command response
        ]
        await applet.sd_spi_iface.initialize()
