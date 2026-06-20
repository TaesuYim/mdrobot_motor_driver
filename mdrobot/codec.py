"""byte / word / long 인코딩 helper.

바이트 순서 규칙(CLAUDE.md §2.2, doc 03):
- 16비트 word는 wire에서 big-endian (DH DL).
- 32비트 long은 low word first, 각 word는 big-endian.
  예: 0x12345678 -> words [0x5678, 0x1234] -> wire 56 78 12 34.
- 독립 word 두 개(예: PNT speed1, speed2)에는 word-swap을 적용하지 않는다.

Python/C++ helper 이름을 doc 03 §11과 맞춘다.
"""

from __future__ import annotations


def u16(value: int) -> int:
    """16비트 unsigned로 마스킹한다."""
    return value & 0xFFFF


def int16(word: int) -> int:
    """16비트 word를 signed int로 해석한다(2의 보수)."""
    word &= 0xFFFF
    return word - 0x10000 if word & 0x8000 else word


def word_from_int16(value: int) -> int:
    """signed 값을 16비트 word(2의 보수)로 인코딩한다."""
    return value & 0xFFFF


def split_u32_low_word_first(value: int) -> tuple[int, int]:
    """32비트 값을 (low_word, high_word) 순서의 word 두 개로 나눈다."""
    value &= 0xFFFFFFFF
    return value & 0xFFFF, (value >> 16) & 0xFFFF


def split_i32_low_word_first(value: int) -> tuple[int, int]:
    """signed 32비트 값을 (low_word, high_word)로 나눈다(인코딩은 unsigned와 동일)."""
    return split_u32_low_word_first(value)


def join_u32_low_word_first(low_word: int, high_word: int) -> int:
    """low/high word를 합쳐 unsigned 32비트 값으로 만든다."""
    return ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)


def join_i32_low_word_first(low_word: int, high_word: int) -> int:
    """low/high word를 합쳐 signed 32비트 값으로 만든다(2의 보수)."""
    raw = join_u32_low_word_first(low_word, high_word)
    return raw - 0x100000000 if raw & 0x80000000 else raw
