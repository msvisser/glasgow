import enum
import logging

import amaranth.lib.crc.catalog
from glasgow.applet import GlasgowAppletError, GlasgowAppletV2
from glasgow.applet.interface.spi_controller import SPIControllerInterface
from glasgow.support.bits import bits
from glasgow.support.bitstruct import bitstruct

__all__ = ["SdSpiError", "SdSpiInterface"]

crc7 = staticmethod(amaranth.lib.crc.catalog.CRC7_MMC(data_width=8).compute)
crc16 = staticmethod(amaranth.lib.crc.catalog.CRC16_XMODEM(data_width=8).compute)


class SdSpiError(GlasgowAppletError):
    pass


class SdCardVersion(enum.Enum):
    SDV1 = enum.auto()
    SDV2 = enum.auto()
    SDHC = enum.auto()


SdCardSpecificData = bitstruct("SdCardSpecificData", 128, [
    (None, 1),
    ("checksum", 7),
    (None, 118),
    ("version", 2),
])

SdCardSpecificDataV1 = bitstruct("SdCardSpecificDataV1", 128, [
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
])

SdCardSpecificDataV2 = bitstruct("SdCardSpecificDataV2", 128, [
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
])


SdCardIdentification = bitstruct("SdCardIdentification", 128, [
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
])


class SdSpiInterface:
    def __init__(self, logger: logging.Logger, spi_iface: SPIControllerInterface):
        self._spi_iface = spi_iface
        self._logger = logger
        self._level = logging.DEBUG if self._logger.name == __name__ else logging.TRACE
        self._ident = "SD SPI"

    def _log(self, message, *args):
        self._logger.log(self._level, self._ident + ": " + message, *args)

    async def _read_byte(self):
        """Read a single byte from the SD card."""
        result = await self._spi_iface.exchange([0xFF])
        return result[0]

    async def _read_bytes(self, length):
        """Read multiple bytes from the SD card."""
        result = await self._spi_iface.exchange([0xFF] * length)
        return bytes(result)

    async def _wakeup(self):
        """Wake up the SD card by sending 74 clock cycles."""
        async with self._spi_iface.select():
            await self._spi_iface.dummy(74)

    async def _wait_until_ready(self):
        """Wait until the SD card is ready."""
        for _ in range(8):
            status = await self._read_byte()
            if status == 0xFF:
                return

        raise SdSpiError("Timeout waiting for SD card to be ready")

    async def _wait_until_response(self):
        """Wait until the SD card responds."""
        for _ in range(8):
            status = await self._read_byte()
            if (status & 0x80) == 0:
                return status

        raise SdSpiError("Timeout waiting for SD card response")

    async def _command_internal(self, command, arg=0, response_length=0):
        """Send a command to the SD card and return the response, without selecting the SPI interface."""
        request = [
            0x40 | (command & 0x3F),
            (arg >> 24) & 0xFF,
            (arg >> 16) & 0xFF,
            (arg >> 8) & 0xFF,
            arg & 0xFF,
        ]
        crc = crc7(request)
        request_with_crc = request + [(crc << 1) | 1]

        await self._wait_until_ready()
        await self._spi_iface.write(request_with_crc)
        status = await self._wait_until_response()

        if status > 1:
            raise SdSpiError(f"Command {command} failed with status {status}")

        if response_length > 0:
            response = await self._read_bytes(response_length)
        else:
            response = bytes()

        return status, response

    async def _command(self, command, arg=0, response_length=0):
        """Send a command to the SD card and return the response."""
        async with self._spi_iface.select():
            status, response = await self._command_internal(
                command, arg, response_length
            )

        return status, response

    async def _app_command(self, command, arg=0, response_length=0):
        """Send an application-specific command to the SD card."""
        status, _ = await self._command(55)
        if status > 1:
            raise SdSpiError(f"App command prefix failed with status {status}")

        return await self._command(command, arg, response_length)

    async def _read_data_block(self, length) -> bytes:
        """Read a data block of the specified length from the SD card."""
        # Wait for data token (0xFE)
        for _ in range(10000):
            token = await self._read_byte()
            if token == 0xFE:
                break
        else:
            raise SdSpiError("Timeout waiting for data token")

        data_and_crc = await self._read_bytes(length + 2)

        data = data_and_crc[:-2]
        crc = (data_and_crc[-2] << 8) | data_and_crc[-1]
        calculated_crc = crc16(data)
        if crc != calculated_crc:
            raise SdSpiError(
                f"CRC mismatch: received {crc:04x}, calculated {calculated_crc:04x}"
            )

        return bytes(data)

    async def _command_with_data_block(
        self, command, arg=0, response_length=0, data_block_length=512
    ) -> bytes:
        """Send a command that expects a data block response."""
        async with self._spi_iface.select():
            status, _ = await self._command_internal(command, arg, response_length)

            if status != 0:
                raise SdSpiError(f"Command {command} failed with status {status}")

            data = await self._read_data_block(data_block_length)

        return data

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
        status, response = await self._command(8, arg=0x000001AA, response_length=4)
        if status == 5:
            self._log("CMD8: SEND_IF_COND -> Illegal command (SD v1.x or MMC)")
            sd_version = 1
        elif status == 1:
            if response != bytes([0x00, 0x00, 0x01, 0xAA]):
                raise SdSpiError(
                    f"CMD8: SEND_IF_COND -> Invalid response {response.hex()}"
                )
            self._log("CMD8: SEND_IF_COND -> OK (SD v2.x)")
            sd_version = 2
        else:
            raise SdSpiError(f"CMD8: SEND_IF_COND failed with status {status}")

        # ACMD41: SD_SEND_OP_COND (with HCS bit for SD v2.x)
        arg = 0x40000000 if sd_version == 2 else 0x00000000
        for _ in range(100):
            status, _ = await self._app_command(41, arg=arg)
            if status == 0:
                break
        else:
            raise SdSpiError(
                "ACMD41: SD_SEND_OP_COND -> Timeout waiting for card to be ready"
            )
        self._log("ACMD41: SD_SEND_OP_COND -> OK")

        # CMD58: READ_OCR
        status, ocr = await self._command(58, response_length=4)
        if status != 0:
            raise SdSpiError(f"CMD58: READ_OCR failed with status {status}")
        self._log(f"CMD58: READ_OCR -> OK, OCR={ocr.hex()}")

        if sd_version == 2 and (ocr[0] & 0x40):
            self._log("Card is SDHC/SDXC")
            return SdCardVersion.SDHC
        elif sd_version == 2:
            self._log("Card is SDSC (v2.x)")
            return SdCardVersion.SDV2
        else:
            self._log("Card is SDSC (v1.x)")
            return SdCardVersion.SDV1

    async def read_card_specific_data(self):
        """Read the card specific data (CSD) from the SD card."""
        data = await self._command_with_data_block(9, data_block_length=16)

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

        self._log(f"CSD: {data.hex()}")

        return csd

    async def read_card_identification(self):
        """Read the card identification (CID) from the SD card."""
        data = await self._command_with_data_block(10, data_block_length=16)

        data_rev = bytes(reversed(data))
        calculated_checksum = crc7(data[:15])
        cid = SdCardIdentification.from_bytes(data_rev)
        if cid.checksum != calculated_checksum:
            raise SdSpiError("Invalid checksum for SdCardIdentification")

        self._log(f"CID: {data.hex()}")

        return cid

    async def read_block(self, address: int) -> bytes:
        """Read a 512-byte block from the SD card at the specified address."""
        self._log(f"Reading block at address {address:#x}")
        data = await self._command_with_data_block(
            17, arg=address, data_block_length=512
        )
        self._log(f"Read block at address {address:#x} -> OK")
        return data

    async def read_blocks(self, start_block: int, count: int) -> bytes:
        """Read multiple 512-byte blocks from the SD card starting at the specified address."""
        self._log(f"Reading {count} blocks starting at address {start_block:#x}")

        async with self._spi_iface.select():
            status, _ = await self._command_internal(18, start_block)
            if status != 0:
                raise SdSpiError(f"CMD failed with status {status}")

            data = bytearray()
            for block_index in range(count):
                data.extend(await self._read_data_block(512))
                print(f"\rRead block {start_block + block_index} -> OK", end="")

            status, _ = await self._command_internal(12)
            if status != 0:
                raise SdSpiError(f"CMD12 failed with status {status}")

            print("\r", end="")

        return bytes(data)


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
            self.spi_iface = SPIControllerInterface(
                self.logger,
                self.assembly,
                cs=args.cs,
                sck=args.sck,
                copi=args.copi,
                cipo=args.cipo,
                mode=3,
            )
            self.sd_spi_iface = SdSpiInterface(self.logger, self.spi_iface)

    async def setup(self, args):
        await self.spi_iface.clock.set_frequency(500e3)

    @classmethod
    def add_run_arguments(cls, parser):
        p_operation = parser.add_subparsers(
            dest="operation", metavar="OPERATION", required=True
        )

        p_identify = p_operation.add_parser("identify", help="identify the SD card")

        p_read = p_operation.add_parser("read", help="read data from the SD card")
        p_read.add_argument("block_address", type=int, help="block address to read from")

        p_read_multiple = p_operation.add_parser("read-multiple", help="read multiple blocks from the SD card")
        p_read_multiple.add_argument("block_address", type=int, help="block address to read from")
        p_read_multiple.add_argument("count", type=int, help="number of blocks to read")

    async def run(self, args):
        try:
            sd_card_version = await self.sd_spi_iface.initialize()

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

                self.logger.info(f"    Max data transfer rate: {card_specific_data.max_data_transfer_rate // 2}MHz")
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
                self.logger.info(f"Reading from address {args.block_address:#x}")
                data = await self.sd_spi_iface.read_block(args.block_address)
                print(data.hex())
                with open(f"sd_block_{args.block_address:08x}.bin", "wb") as f:
                    f.write(data)
            elif args.operation == "read-multiple":
                self.logger.info(f"Reading {args.count} blocks starting from address {args.block_address:#x}")
                data = await self.sd_spi_iface.read_blocks(args.block_address, args.count)

                for offset in range(0, len(data), 32):
                    chunk = data[offset:offset+32]
                    hex_chunk = ' '.join(f"{byte:02x}" for byte in chunk)
                    print(f"{args.block_address * 512 + offset:08x}  {hex_chunk}")


        except SdSpiError as e:
            self.logger.error("Error: %s", e)
        finally:
            await self.device.set_voltage("AB", 0.0)
