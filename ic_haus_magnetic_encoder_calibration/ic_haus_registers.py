"""iC-MU register definitions.

Defines ``ICHausRegister`` / ``ICHausRegisterField`` descriptors for every
iC-MU register used in calibration, together with the ``BissAction`` enum
that drives the BiSS bidirectional interface.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Tuple

# ---------------------------------------------------------------------------
# BiSS bidirectional CTL actions
# ---------------------------------------------------------------------------


class BissAction(IntEnum):
    """BiSS bidirectional interface control actions."""

    NO_ACTION = 0
    READ = 1
    WRITE = 2


# ---------------------------------------------------------------------------
# Register field / register descriptors
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ICHausRegisterField:
    """A named bitfield inside an iC-MU register.

    Args:
        mask: Bitmask selecting the field bits (already shifted).
        shift: Number of bits the field is shifted from bit-0.
        name: Human-readable description of the field.
    """

    mask: int
    shift: int
    name: str = ""

    @classmethod
    def from_bits(cls, *, low: int, high: int, name: str = "") -> "ICHausRegisterField":
        """Create a field spanning bits [low, high] (inclusive).

        Args:
            low: Lowest bit index (0-based).
            high: Highest bit index (0-based, inclusive).
            name: Human-readable description of the field.

        Returns:
            A new ICHausRegisterField with the computed mask and shift.
        """
        mask = sum(1 << i for i in range(low, high + 1))
        return cls(mask=mask, shift=low, name=name)

    def extract(self, raw: int) -> int:
        """Extract this field's value from a raw register byte.

        Args:
            raw: Full 8-bit register value.

        Returns:
            Field value, right-shifted to bit-0.
        """
        return (raw & self.mask) >> self.shift

    def insert(self, raw: int, value: int) -> int:
        """Return *raw* with this field replaced by *value*.

        Args:
            raw: Original 8-bit register value.
            value: New field value (unshifted, will be shifted internally).

        Returns:
            Updated register byte.
        """
        return (raw & ~self.mask) | ((value << self.shift) & self.mask)


class ICHausRegister:
    """iC-MU register descriptor with optional sub-fields.

    Simple registers (e.g. GX_M at 0x01) have no sub-fields.
    Compound registers (e.g. OUT_MSB_ZERO at 0x11) declare named fields
    via keyword arguments.

    Args:
        address: iC-MU register address (0x00-0x7F).
        name: Human-readable register description.
        **fields: Named ``ICHausRegisterField`` instances.
    """

    __slots__ = ("address", "name", "_fields")

    def __init__(self, *, address: int, name: str = "", **fields: ICHausRegisterField) -> None:
        self.address: int = address
        self.name: str = name
        self._fields: Dict[str, ICHausRegisterField] = fields

    # -- Field access --

    def field(self, field_name: str) -> ICHausRegisterField:
        """Return the ICHausRegisterField for *field_name*.

        Args:
            field_name: Field name as passed to the constructor.

        Returns:
            The ICHausRegisterField descriptor.

        Raises:
            KeyError: If no field with that name exists.
        """
        return self._fields[field_name]

    @property
    def field_names(self) -> Tuple[str, ...]:
        """Return the names of all declared fields."""
        return tuple(self._fields)


# ---------------------------------------------------------------------------
# iC-MU register instances
# ---------------------------------------------------------------------------

# Analog track adjustments (simple 8-bit, no sub-fields)
GX_M = ICHausRegister(address=0x01, name="Cosine gain master")
VOSS_M = ICHausRegister(address=0x02, name="Sine offset master")
VOSC_M = ICHausRegister(address=0x03, name="Cosine offset master")
PH_M = ICHausRegister(address=0x04, name="Phase master")
GX_N = ICHausRegister(address=0x07, name="Cosine gain nonius")
VOSS_N = ICHausRegister(address=0x08, name="Sine offset nonius")
VOSC_N = ICHausRegister(address=0x09, name="Cosine offset nonius")
PH_N = ICHausRegister(address=0x0A, name="Phase nonius")

# Configuration registers with sub-fields
ENAC = ICHausRegister(
    address=0x05,
    name="Enable / auto-calibrate",
    enac=ICHausRegisterField.from_bits(low=7, high=7, name="Auto-calibrate enable"),
)
MODEA_MODEB = ICHausRegister(
    address=0x0B,
    name="Mode A / Mode B",
    modea=ICHausRegisterField.from_bits(low=0, high=2, name="Interface mode A"),
)
CFGEW = ICHausRegister(
    address=0x0C,
    name="Status config for E/W bits",
)
MPC = ICHausRegister(
    address=0x0F,
    name="Master periods per revolution",
    mpc=ICHausRegisterField.from_bits(low=0, high=3, name="Master period count exponent"),
)
OUT_MSB_ZERO = ICHausRegister(
    address=0x11,
    name="Output MSB / zero",
    out_msb=ICHausRegisterField.from_bits(low=0, high=4, name="Output resolution MSB"),
    out_zero=ICHausRegisterField.from_bits(low=5, high=7, name="Zero correction"),
)
OUT_LSB_ST = ICHausRegister(
    address=0x12,
    name="Output LSB / status",
    out_lsb=ICHausRegisterField.from_bits(low=0, high=3, name="Output resolution LSB"),
    mode_st=ICHausRegisterField.from_bits(low=4, high=5, name="Status output mode"),
)
TEST = ICHausRegister(address=0x18, name="Test mode")

# SPO (nonius offset) registers
# The iC-MU packs two 4-bit SPO values per 8-bit register:
#   low nibble [3:0] = odd-indexed offset, high nibble [7:4] = even-indexed offset.
# Register 0x52 is special: low = SPO_BASE, high = SPO_0.
SPO_REGISTERS: List[ICHausRegister] = [
    ICHausRegister(
        address=0x52,
        name="SPO base / offset 0",
        spo_base=ICHausRegisterField.from_bits(low=0, high=3, name="SPO base value"),
        spo_0=ICHausRegisterField.from_bits(low=4, high=7, name="SPO offset 0"),
    ),
    *[
        ICHausRegister(
            address=0x53 + i,
            name=f"SPO offsets {2 * i + 1}-{2 * i + 2}",
            **{
                f"spo_{2 * i + 1}": ICHausRegisterField.from_bits(
                    low=0,
                    high=3,
                    name=f"SPO offset {2 * i + 1}",
                ),
                f"spo_{2 * i + 2}": ICHausRegisterField.from_bits(
                    low=4,
                    high=7,
                    name=f"SPO offset {2 * i + 2}",
                ),
            },
        )
        for i in range(7)
    ],
]

# Read-only / command / status
HARD_REV = ICHausRegister(address=0x74, name="Hardware revision")
CMD = ICHausRegister(address=0x75, name="Command register")
STATUS1 = ICHausRegister(
    address=0x77,
    name="Status register 1",
    epr_err=ICHausRegisterField.from_bits(low=6, high=6, name="EEPROM error"),
    crc_err=ICHausRegisterField.from_bits(low=7, high=7, name="CRC error"),
)
