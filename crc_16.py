# CRC16 Lookup Table
crc_tab16 = [0] * 256
crc_tab16_init = 0

CRC_START_16 = 0xFFFF  # Assuming CRC_START_16 is 0xFFFF, adjust if different
CRC_START_MODBUS = 0xFFFF  # Assuming CRC_START_MODBUS is 0xFFFF, adjust if different
CRC_POLY_16 = 0xA001  # Assuming CRC_POLY_16 is 0xA001, adjust if different

def init_crc16_tab():
    global crc_tab16_init
    global crc_tab16
    for i in range(256):
        crc = 0
        c = i
        for j in range(8):
            if (crc ^ c) & 0x0001:
                crc = (crc >> 1) ^ CRC_POLY_16
            else:
                crc >>= 1
            c >>= 1
        crc_tab16[i] = crc
    crc_tab16_init = 1

def crc16_check(input_str):
    global crc_tab16_init
    global crc_tab16
    if not crc_tab16_init:
        init_crc16_tab()
    crc = CRC_START_16
    for byte in input_str:
        crc = (crc >> 8) ^ crc_tab16[(crc ^ byte) & 0x00FF]
    return crc

# input_bytes = bytes([0xA5, 0x04, 0x00, 0xF8, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11])
# # 计算CRC-16
# crc_result = crc_16(input_bytes)
# input_bytes_with_crc = input_bytes + bytes([crc_result & 0xFF,(crc_result >> 8) & 0xFF])
# # 打印结果
# print("包含CRC的input_bytes:", list(input_bytes_with_crc))
