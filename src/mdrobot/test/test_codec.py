"""byte/word/long helper unit tests."""

from mdrobot.codec import (
    int16,
    join_i32_low_word_first,
    join_u32_low_word_first,
    split_i32_low_word_first,
    split_u32_low_word_first,
    u16,
    word_from_int16,
)


def test_u16_masks():
    assert u16(0x12345) == 0x2345
    assert u16(-1) == 0xFFFF


def test_int16_signed_interpretation():
    assert int16(0x0064) == 100
    assert int16(0xFF9C) == -100
    assert int16(0x8000) == -32768
    assert int16(0x7FFF) == 32767


def test_word_from_int16_twos_complement():
    assert word_from_int16(100) == 0x0064
    assert word_from_int16(-100) == 0xFF9C
    assert word_from_int16(-1) == 0xFFFF


def test_split_low_word_first():
    # 0x12345678 -> [0x5678, 0x1234]
    assert split_u32_low_word_first(0x12345678) == (0x5678, 0x1234)
    assert split_i32_low_word_first(0x12345678) == (0x5678, 0x1234)


def test_join_low_word_first():
    assert join_u32_low_word_first(0x5678, 0x1234) == 0x12345678
    assert join_i32_low_word_first(0x5678, 0x1234) == 0x12345678


def test_long_signed_roundtrip():
    for value in (0, 1, -1, 100, -100, 0x12345678, -2, 2147483647, -2147483648):
        low, high = split_i32_low_word_first(value)
        assert join_i32_low_word_first(low, high) == value


def test_join_i32_negative():
    # -2 -> 0xFFFFFFFE -> low 0xFFFE, high 0xFFFF
    assert split_i32_low_word_first(-2) == (0xFFFE, 0xFFFF)
    assert join_i32_low_word_first(0xFFFE, 0xFFFF) == -2
