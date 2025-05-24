def crc8(initial, poly, data: bytes) -> int:
    """
    Calculate CRC-8 with polynomial 0x07, initial value 0x00.
    :param data: Input data as bytes or bytearray
    :return: CRC-8 value as integer
    """
    crc = initial
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def xor_checksum(data: bytes) -> int:
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum

def rotate_left_checksum(data: bytes) -> int:
    checksum = 0
    for b in data:
        checksum = ((checksum << 1) | (checksum >> 7)) & 0xFF
        checksum = (checksum + b) & 0xFF
    return checksum

def fletcher8_checksum(data: bytes) -> int:
    sum1 = 0
    sum2 = 0
    for b in data:
        sum1 = (sum1 + b) % 15
        sum2 = (sum2 + sum1) % 15
    return (sum2 << 4) | sum1

def adler8_checksum(data: bytes) -> int:
    a = 1
    b = 0
    for byte in data:
        a = (a + byte) % 9
        b = (b + a) % 9
    return (b << 4) | a

functions = {
    "xor": xor_checksum,
    "rotate_left": rotate_left_checksum,
    "fletcher8": fletcher8_checksum,
    "adler8": adler8_checksum
}

# private byte check(byte[] buf, byte len)
# {
# byte num1 = 0;
# int num2 = 0;
# for (byte index = 0; (int) index < (int) len; ++index)
# {
# num1 ^= buf[(int) index];
# num2 += (int) buf[(int) index];
# }
# return (byte) (((int) num1 ^ num2) & (int) byte.MaxValue);
# }

def custom_check(buf: bytes, length: int) -> int:
    num1 = 0
    num2 = 0
    for i in range(length):
        num1 ^= buf[i]
        num2 += buf[i]
    num2 &= 0xFF
    return (num1 ^ num2) & 0xFF

# Example usage:
if __name__ == "__main__":
    msg = bytes([0x7E, 0x01, 0x01, 0x00])
    msg2 = bytes([0x7E, 0x01, 0x33, 0x00])
    msg3 = bytes([0x7e, 0x01, 0x42, 0x00])

    # for poly in range(0, 256):
    #     for initial in range(0, 256):
    #         checksum = crc8(initial, poly, msg)
    #         #print(f"CRC-8: {checksum:02X}")
    #         if checksum == 0xFE:
    #             print(f"Polynomial: {poly:02X} Initial: {initial:02X} Checksum: {checksum:02X}")
    #             # Try with different message
    #             checksum2 = crc8(initial, poly, msg2)
    #             if checksum2 == 0xFE:
    #                 print(f"Matching Polynomial: {poly:02X} Initial: {initial:02X} Checksum2: {checksum2:02X}")
    # for name, func in functions.items():
    #     checksum = func(msg)
    #     print(f"{name.capitalize()} Checksum: {checksum:02X}")
    #     checksum2 = func(msg2)
    #     print(f"{name.capitalize()} Checksum2: {checksum2:02X}")
    checksum = custom_check(msg, len(msg))
    print(f"Custom Checksum: {checksum:02X}")
    checksum2 = custom_check(msg2, len(msg2))
    print(f"Custom Checksum2: {checksum2:02X}")
    checksum3 = custom_check(msg3, len(msg3))
    print(f"Custom Checksum3: {checksum3:02X}")
    print("Done")

    