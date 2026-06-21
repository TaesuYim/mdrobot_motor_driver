"""mdrobot communication-layer exceptions.

Raise distinguishable errors for ID mismatch, function mismatch, byte-count
mismatch, CRC error, short response, and Modbus exception responses.
"""


class MdrobotError(Exception):
    """Base class for all mdrobot communication exceptions."""


class CrcError(MdrobotError):
    """The CRC16 of a response frame does not match."""


class ProtocolError(MdrobotError):
    """Frame structure / echo / exception and other protocol-level errors.

    For a Modbus exception response, the function and exception code are attached.
    """

    def __init__(self, message: str, *, function: int | None = None, code: int | None = None) -> None:
        super().__init__(message)
        self.function = function
        self.code = code


class IncompleteResponseError(ProtocolError):
    """Fewer bytes arrived than expected (partial response / timeout)."""
