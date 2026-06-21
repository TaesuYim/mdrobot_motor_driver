"""byte / word / long encoding helpers.

Byte-order rules:
- a 16-bit word is big-endian on the wire (DH DL).
- a 32-bit long is low word first, each word big-endian.
  e.g. 0x12345678 -> words [0x5678, 0x1234] -> wire 56 78 12 34.
- two independent words (e.g. dual-channel speed1, speed2) are NOT word-swapped.
"""

from __future__ import annotations


def u16(value: int) -> int:
    """Mask to 16-bit unsigned."""
    return value & 0xFFFF


def int16(word: int) -> int:
    """Interpret a 16-bit word as a signed int (two's complement)."""
    word &= 0xFFFF
    return word - 0x10000 if word & 0x8000 else word


def word_from_int16(value: int) -> int:
    """Encode a signed value as a 16-bit word (two's complement)."""
    return value & 0xFFFF


def split_u32_low_word_first(value: int) -> tuple[int, int]:
    """Split a 32-bit value into two words: (low_word, high_word)."""
    value &= 0xFFFFFFFF
    return value & 0xFFFF, (value >> 16) & 0xFFFF


def split_i32_low_word_first(value: int) -> tuple[int, int]:
    """Split a signed 32-bit value into (low_word, high_word) (same encoding as unsigned)."""
    return split_u32_low_word_first(value)


def join_u32_low_word_first(low_word: int, high_word: int) -> int:
    """Join low/high words into an unsigned 32-bit value."""
    return ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)


def join_i32_low_word_first(low_word: int, high_word: int) -> int:
    """Join low/high words into a signed 32-bit value (two's complement)."""
    raw = join_u32_low_word_first(low_word, high_word)
    return raw - 0x100000000 if raw & 0x80000000 else raw
