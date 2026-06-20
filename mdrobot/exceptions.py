"""mdrobot 통신 계층 예외.

doc 02 §9 / doc 07 §4.1: ID 불일치, function 불일치, byte count 불일치, CRC 오류,
응답 길이 부족, Modbus exception response를 서로 구분 가능한 예외로 올린다.
"""


class MdrobotError(Exception):
    """모든 mdrobot 통신 예외의 베이스."""


class CrcError(MdrobotError):
    """응답 프레임의 CRC16이 맞지 않는다."""


class ProtocolError(MdrobotError):
    """프레임 구조/echo/exception 등 프로토콜 수준 오류.

    Modbus exception response인 경우 function과 exception code를 함께 담는다.
    """

    def __init__(self, message: str, *, function: int | None = None, code: int | None = None) -> None:
        super().__init__(message)
        self.function = function
        self.code = code


class IncompleteResponseError(ProtocolError):
    """기대한 길이만큼 바이트가 도착하지 않았다(부분 응답/타임아웃 후보)."""
