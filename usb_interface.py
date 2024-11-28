import time
import struct
from crc_8 import crc8_check
from crc_16 import crc16_check
import serial
import serial.tools.list_ports

class SerialCommunication:
    def __init__(self, target_device=None, baudrate=921600, timeout=1):
        self.target_device = target_device
        self.baudrate = baudrate
        self.timeout = timeout
        self.com_port = None

    def find_com_port(self, target_name=None):
        """
        查找目标COM口，如果指定了 target_name，则返回匹配的COM口。
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"发现端口: {port.device}, 描述: {port.description}")
            if target_name and target_name in port.description:
                print(f"找到目标端口: {port.device}")
                return port.device
        return None if target_name else [port.device for port in ports]

    def find_com_port_by_description(self, description):
        """
        根据设备描述查找 COM 端口
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if description in port.description:
                return port.device
        return None  # 如果没有找到匹配的 COM 端口，则返回 None

    def send_data(self, data):
        """
        发送数据到指定的串口
        """
        if not self.com_port:
            raise ValueError("COM端口未找到。")
        
        try:
            with serial.Serial(self.com_port, baudrate=self.baudrate, timeout=self.timeout) as ser:
                ser.write(data)
                print(f"发送数据: {data.hex().upper()}")
                response = ser.read(ser.in_waiting or 1)
                print(f"接收数据: {response.hex().upper() if response else '无数据'}")
                return response
        except serial.SerialException as e:
            print(f"串口操作失败: {e}")
            return None

    def float_to_byte_array(self, float_list):
        """
        将浮点数列表转换为字节数组
        """
        byte_array = bytearray()
        for f in float_list:
            byte_array.extend(struct.pack('<f', f))  # 小端格式浮点数
        return byte_array

    def bytes_to_float(self, byte_array):
        """
        将字节数组转换回浮点数
        """
        if len(byte_array) % 4 != 0:
            raise ValueError("字节数组长度必须是4的倍数！")
        return [struct.unpack('<f', byte_array[i*4:i*4+4])[0] for i in range(len(byte_array)//4)]

    def tx_data_format(self, float_list):
        """
        格式化要发送的数据
        """
        tx_data = bytes([0xA5])  # 帧头
        len_bytes = len(float_list) * 4  # 计算数据长度
        if len_bytes > 0x05dc or len_bytes <= 0:
            return []
        tx_data += bytes([len_bytes & 0xFF, (len_bytes >> 8) & 0xFF])  # 数据长度
        tx_data += bytes([crc8_check(tx_data, 3) & 0xFF])  # CRC-8校验
        tx_data += bytes([11 & 0xFF, 11 & 0xFF])  # 命令码
        tx_data += self.float_to_byte_array(float_list)  # 添加浮动数据
        crc_result = crc16_check(tx_data)  # CRC-16校验
        tx_data += bytes([crc_result & 0xFF, (crc_result >> 8) & 0xFF])
        return tx_data

    def start_sending(self, float_list, interval=1000):
        """
        开始定时发送数据
        """
        if not self.com_port:
            print("未找到目标COM口，无法发送数据。")
            return

        try:
            with serial.Serial(self.com_port, baudrate=self.baudrate, timeout=self.timeout) as ser:
                print(f"开始以 {interval}Hz 发送数据到 {self.com_port}...")
                while True:
                    start_time = time.perf_counter()
                    float_list[0] += 0.1
                    data = self.tx_data_format(float_list)
                    # 发送数据
                    ser.write(data)
                    print(f"发送数据: {data.hex().upper()}")
                    # 接收数据
                    receive_list = self.bytes_to_float(ser.read(size=16))
                    print("接收到的 MCU 数据:", receive_list)

                    # 1000hz 发送，写得很草，想优化（
                    while (time.perf_counter() - start_time) < interval / 1_000_000:
                        pass
        except KeyboardInterrupt:
            print("\n发送中止")

# 示例用法
if __name__ == "__main__":
    serial_comm = SerialCommunication(target_device="STM32 Virtual ComPort")
    com_port = serial_comm.find_com_port_by_description("STM32 Virtual ComPort")
    if com_port:
        serial_comm.com_port = com_port
        float_list = [-100.0]  # 初始浮动数据
        serial_comm.start_sending(float_list)
    else:
        print("未找到目标 COM 端口。")
