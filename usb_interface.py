import sys
import time
import numpy as np
import struct
from crc_8 import crc8_check
from crc_16 import crc16_check
import serial
import serial.tools.list_ports

def find_com_port(target_name=None):
    """
    查找目标COM口，如果指定了 target_name，则返回匹配的COM口。
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # 打印可用端口信息
        print(f"发现端口: {port.device}, 描述: {port.description}")
        if target_name and target_name in port.description:
            print(f"找到目标端口: {port.device}")
            return port.device
    return None if target_name else [port.device for port in ports]

def send_data(com_port, data, baudrate=921600, timeout=1):
    """
    param baudrate: 波特率，默认为 921600
    param timeout: 超时时间，默认为 1 秒
    """
    try:
        # 打开串口
        with serial.Serial(com_port, baudrate=baudrate, timeout=timeout) as ser:
            # 发送数据
            ser.write(data)
            print(f"发送数据: {data.hex().upper()}")  # 转为十六进制打印发送内容

            # 可选：接收返回数据
            response = ser.read(ser.in_waiting or 1)  # 读取接收缓冲区中的数据
            print(f"接收数据: {response.hex().upper() if response else '无数据'}")
            return response
    except serial.SerialException as e:
        print(f"串口操作失败: {e}")
        return None

def float_to_byte_array(float_list, num_floats):
    # 初始化一个空字节数组，每个浮点数占用4字节（32位）
    byte_array = bytearray()
    
    # 确保列表中的浮点数个数不超过所需个数
    num_floats = min(num_floats, len(float_list))
    
    # 将浮点数转换为字节并添加到数组中
    for i in range(num_floats):
        # struct.pack将浮点数转换为4字节的字节数组
        # '>f' 表示大端格式的单精度浮点数（4字节）
        byte_array.extend(struct.pack('<f', float_list[i]))
    
    return byte_array

def bytes_to_float(byte_array):
    float_value=[]
    if len(byte_array)%4 != 0:
        raise ValueError("输入的字节数组长度必须为4！")
    for i in range (len(byte_array)//4):
        float_value+=[struct.unpack('<f', byte_array[i*4:i*4+4])[0]]  # '<f' 表示小端序浮点数
    return float_value

def tx_data_format(float_list):
    tx_data=bytes([0xA5]) #指定帧头 1
    len_bytes=len(float_list) *4 #float32 单精度
    # print(len_bytes)
    # USB2.0 全速12M bit/s 过长的数据是不合理的！！
    if len_bytes>0x05dc or len_bytes<=0:
        return []
    # 添加数据长度 2~3
    tx_data+=bytes([len_bytes & 0xFF,(len_bytes >> 8) & 0xFF])
    # crc—8校验 4
    tx_data+=bytes([crc8_check(tx_data,3)& 0xFF])
    # 添加目前无用的命令码,或许后续有用（）
    tx_data+=bytes([11& 0xFF,11& 0xFF])
    # f32塞入发送列表
    tx_data+=float_to_byte_array(float_list,len(float_list))
    # crc-16校验 2byte
    crc_result = crc16_check(tx_data)
    tx_data+=bytes([crc_result & 0xFF,(crc_result >> 8) & 0xFF])
    return tx_data

if __name__ == "__main__":
    float_list = []
    receive_list=[]
    k=-100.0
    float_list.append(k)
    a=tx_data_format(float_list)

    target_device = "USB 串行设备"  # 替换为你的设备描述（如 "Arduino" 或 "CH340"）
    com_port = find_com_port(target_device)
    if not com_port:
        print("未找到目标COM口，列出所有可用端口。")
        available_ports = find_com_port()
        print("可用端口: ", available_ports)
        if available_ports:
            com_port = available_ports[0]  # 手动选择第一个端口
    # 发送数据
    if com_port:
        interval = 1000  # 1 kHz 
        try:
            with serial.Serial(com_port, baudrate=921600, timeout=1) as ser:
                print(f"开始以 1kHz 发送数据到 {com_port}...")
                while True:
                    start_time = time.perf_counter()
                    k+=0.1
                    float_list[0]=k
                    a=tx_data_format(float_list)
                    # 发送数据
                    ser.write(a)
                    # print(f"发送数据: {a.hex().upper()}")
                    # print(" ".join(f"{byte:02x}" for byte in data))
                    receive_list=bytes_to_float(ser.read(size=16))
                    print(receive_list)
            # 检查是否接收到足够的数据
                    # if len(data) < 16:
                    #     print(f"警告: 仅接收到 {len(data)} 字节数据，可能未达到预期长度 {16}")
                    #精确1000hz发送
                    #有点草可以优化
                    while (time.perf_counter() - start_time) < interval / 1_000_000:
                        pass
        except KeyboardInterrupt:
            print("\n发送中止")
    else:
        print("未找到可用COM口，无法发送数据。")
