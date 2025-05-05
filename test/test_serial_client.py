#!/usr/bin/env python3
import serial
import struct
import time
import random
import math
import argparse
import threading
import sys

# 定义数据包格式
RX_PACKET_SIZE = 32  # 从C++接收的包大小
TX_PACKET_SIZE = 32  # 向C++发送的包大小

class SimpleTestClient:
    """简化版测试客户端，模拟裁判系统和下位机"""
    
    def __init__(self, port, baudrate):
        # 串口设置
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # 数据状态
        self.running = False
        self.paused = False
        self.show_raw_data = False  # 是否显示原始数据
        self.in_bytes_count = 0     # 接收字节计数
        self.out_bytes_count = 0    # 发送字节计数
        
        # 模拟数据
        self.decator_mode = 2          # 默认模式2
        self.roll = 0.0                # 滚转角
        self.pitch_gimbal = 0.0        # 小云台俯仰角
        self.yaw_gimbal = 0.0          # 小云台偏航角
        self.pitch_chassis = 0.0       # 大云台俯仰角
        self.yaw_chassis = 0.0         # 大云台偏航角
        self.game_status = 0           # 比赛状态
        self.remaining_time = 180      # 剩余时间
        self.blood = 400               # 血量
        self.is_free_rebirth = 0       # 是否可自由复活
        self.is_retreating = 0         # 撤退命令
        self.is_drone_avoiding = 0     # 防无人机
        self.is_outpost_attacking = 0  # 攻击前哨站
        
        # 收到的控制命令
        self.last_cmd = {
            'pitch_diff': 0.0,
            'yaw_diff': 0.0,
            'distance': 0.0,
            'fire_advice': 0,
            'vx': 0.0,
            'vy': 0.0,
            'vw': 0.0,
            'move_mode': 0
        }
        
        # 场景控制
        self.active_scenario = None
        self.scenarios = {
            '1': self.scenario_normal_match,         # 正常比赛
            '2': self.scenario_low_health,           # 低血量测试
            '3': self.scenario_emergency_retreat,    # 紧急撤退
            '4': self.scenario_drone_attack,         # 无人机攻击
            '5': self.scenario_match_phases          # 比赛阶段转换
        }
        
        # 数据记录
        self.data_log = []
        self.is_logging = False
        
    def open_serial(self):
        """打开串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            return True
        except Exception as e:
            print(f"打开串口失败: {e}")
            return False
    
    def close_serial(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def send_data(self):
        """发送数据包到串口"""
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            # 创建一个空的字节数组
            data_bytes = bytearray(TX_PACKET_SIZE)
            
            # 按照RxPacket结构体的布局填充数据
            struct.pack_into('<B', data_bytes, 0, self.decator_mode)
            struct.pack_into('<f', data_bytes, 1, self.roll)
            struct.pack_into('<f', data_bytes, 5, self.pitch_gimbal)
            struct.pack_into('<f', data_bytes, 9, self.yaw_gimbal)
            struct.pack_into('<f', data_bytes, 13, self.pitch_chassis)
            struct.pack_into('<f', data_bytes, 17, self.yaw_chassis)
            struct.pack_into('<B', data_bytes, 21, self.game_status)
            struct.pack_into('<H', data_bytes, 22, self.remaining_time)
            struct.pack_into('<H', data_bytes, 24, self.blood)
            struct.pack_into('<B', data_bytes, 26, self.is_free_rebirth)
            struct.pack_into('<B', data_bytes, 27, self.is_retreating)
            struct.pack_into('<B', data_bytes, 28, self.is_drone_avoiding)
            struct.pack_into('<B', data_bytes, 29, self.is_outpost_attacking)
            struct.pack_into('<B', data_bytes, 30, 0)  # 占位符3保留为0
            struct.pack_into('<B', data_bytes, 31, 0)  # 32字节对齐填充
            
            # 发送数据
            bytes_sent = self.ser.write(data_bytes)
            self.out_bytes_count += bytes_sent

            # 记录发送的数据
            if self.is_logging:
                timestamp = time.strftime("%H:%M:%S.%f")[:-3]
                self.data_log.append({
                    'timestamp': timestamp,
                    'direction': 'TX',
                    'raw_data': ' '.join([f'{b:02x}' for b in data_bytes]),
                    'mode': self.decator_mode,
                    'status': self.game_status,
                    'blood': self.blood,
                    'retreating': self.is_retreating,
                    'drone_avoiding': self.is_drone_avoiding,
                })

            # 如果开启了显示原始数据的选项
            if self.show_raw_data:
                hex_data = ' '.join([f'{b:02x}' for b in data_bytes])
                print(f"\n发送: {hex_data}")
                print(f"解析: 模式={self.decator_mode}, 状态={self.game_status}, " +
                      f"血量={self.blood}, 撤退={self.is_retreating}, " +
                      f"无人机={self.is_drone_avoiding}, 字节数={bytes_sent}")

            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False
    
    def receive_data(self):
        """从串口接收数据"""
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            # 检查有多少数据可读
            in_waiting = self.ser.in_waiting
            if in_waiting > 0 and self.show_raw_data:
                print(f"\n串口有 {in_waiting} 字节数据等待读取")
            
            if in_waiting >= RX_PACKET_SIZE:
                rx_data = self.ser.read(RX_PACKET_SIZE)
                self.in_bytes_count += len(rx_data)
                
                if len(rx_data) == RX_PACKET_SIZE:
                    # 解析TxPacket结构体数据
                    try:
                        pitch_diff, yaw_diff, distance, fire_advice, vx, vy, vw, move_mode = struct.unpack('<fffIfff I', rx_data)
                        
                        # 更新收到的命令
                        self.last_cmd['pitch_diff'] = pitch_diff
                        self.last_cmd['yaw_diff'] = yaw_diff
                        self.last_cmd['distance'] = distance
                        self.last_cmd['fire_advice'] = fire_advice
                        self.last_cmd['vx'] = vx
                        self.last_cmd['vy'] = vy
                        self.last_cmd['vw'] = vw
                        self.last_cmd['move_mode'] = move_mode
                        
                        # 记录接收的数据
                        if self.is_logging:
                            timestamp = time.strftime("%H:%M:%S.%f")[:-3]
                            self.data_log.append({
                                'timestamp': timestamp,
                                'direction': 'RX',
                                'raw_data': ' '.join([f'{b:02x}' for b in rx_data]),
                                'pitch_diff': pitch_diff,
                                'yaw_diff': yaw_diff,
                                'distance': distance,
                                'fire_advice': fire_advice,
                                'vx': vx,
                                'vy': vy,
                                'vw': vw,
                                'move_mode': move_mode
                            })
                        
                        # 打印收到的命令
                        if self.show_raw_data:
                            hex_data = ' '.join([f'{b:02x}' for b in rx_data])
                            print(f"\n收到: {hex_data}")
                            print(f"解析: pitch={pitch_diff:.2f}, yaw={yaw_diff:.2f}, " +
                                f"dist={distance:.2f}, fire={fire_advice}, " +
                                f"vx={vx:.2f}, vy={vy:.2f}, vw={vw:.2f}, " +
                                f"mode={move_mode}")
                        else:
                            print(f"\r收到命令: 模式={move_mode}, vx={vx:.2f}, vy={vy:.2f}, vw={vw:.2f}")
                        
                        return True
                    except struct.error as e:
                        if self.show_raw_data:
                            hex_data = ' '.join([f'{b:02x}' for b in rx_data])
                            print(f"\n解析数据失败: {e}")
                            print(f"原始数据: {hex_data}")
                else:
                    if self.show_raw_data:
                        print(f"\n收到数据长度异常: {len(rx_data)} != {RX_PACKET_SIZE}")
            return False
        except Exception as e:
            print(f"接收数据失败: {e}")
            return False
    
    def read_all_data(self):
        """读取串口中所有可用数据，用于调试"""
        if not self.ser or not self.ser.is_open:
            print("\n串口未打开")
            return
            
        try:
            in_waiting = self.ser.in_waiting
            if in_waiting > 0:
                data = self.ser.read(in_waiting)
                hex_data = ' '.join([f'{b:02x}' for b in data])
                print(f"\n读取所有数据: {in_waiting} 字节")
                print(f"数据: {hex_data}")
            else:
                print("\n串口中没有数据")
        except Exception as e:
            print(f"\n读取数据失败: {e}")
    
    def send_test_data(self):
        """发送一个简单的测试数据包"""
        if not self.ser or not self.ser.is_open:
            print("\n串口未打开")
            return False
        
        try:
            # 准备一个明显的测试数据
            test_data = bytearray(TX_PACKET_SIZE)
            for i in range(TX_PACKET_SIZE):
                test_data[i] = i % 256
                
            # 发送数据
            bytes_sent = self.ser.write(test_data)
            self.out_bytes_count += bytes_sent
            
            # 显示发送信息
            hex_data = ' '.join([f'{b:02x}' for b in test_data])
            print(f"\n发送测试数据: {hex_data}")
            print(f"发送了 {bytes_sent} 字节")
            
            return True
        except Exception as e:
            print(f"\n发送测试数据失败: {e}")
            return False
    
    def print_serial_stats(self):
        """打印串口统计信息"""
        print("\n===== 串口统计信息 =====")
        print(f"串口: {self.port}, 波特率: {self.baudrate}")
        
        if not self.ser or not self.ser.is_open:
            print("当前状态: 未打开")
            return
            
        print(f"当前状态: 已打开")
        print(f"等待读取的字节数: {self.ser.in_waiting}")
        print(f"总接收字节数: {self.in_bytes_count}")
        print(f"总发送字节数: {self.out_bytes_count}")
    
    def print_data_structure(self):
        """打印当前数据结构的详细信息"""
        print("\n===== 数据结构信息 =====")
        print(f"TX包大小: {TX_PACKET_SIZE} 字节")
        print(f"RX包大小: {RX_PACKET_SIZE} 字节")
        print("\n发送数据结构:")
        print(f"  decator_mode (uint8_t): {self.decator_mode}")
        print(f"  roll (float): {self.roll}")
        print(f"  pitch_gimbal (float): {self.pitch_gimbal}")
        print(f"  yaw_gimbal (float): {self.yaw_gimbal}")
        print(f"  pitch_chassis (float): {self.pitch_chassis}")
        print(f"  yaw_chassis (float): {self.yaw_chassis}")
        print(f"  game_status (uint8_t): {self.game_status}")
        print(f"  remaining_time (uint16_t): {self.remaining_time}")
        print(f"  blood (uint16_t): {self.blood}")
        print(f"  is_free_rebirth (uint8_t): {self.is_free_rebirth}")
        print(f"  is_retreating (uint8_t): {self.is_retreating}")
        print(f"  is_drone_avoiding (uint8_t): {self.is_drone_avoiding}")
        print(f"  is_outpost_attacking (uint8_t): {self.is_outpost_attacking}")
        print("\n接收数据结构:")
        print(f"  pitch_diff (float): {self.last_cmd['pitch_diff']}")
        print(f"  yaw_diff (float): {self.last_cmd['yaw_diff']}")
        print(f"  distance (float): {self.last_cmd['distance']}")
        print(f"  fire_advice (uint32_t): {self.last_cmd['fire_advice']}")
        print(f"  vx (float): {self.last_cmd['vx']}")
        print(f"  vy (float): {self.last_cmd['vy']}")
        print(f"  vw (float): {self.last_cmd['vw']}")
        print(f"  move_mode (uint32_t): {self.last_cmd['move_mode']}")
        print("=======================")
    
    def update_data(self):
        """更新模拟数据值"""
        if self.paused:
            return
            
        # 默认行为：生成小幅度的姿态变化
        self.roll = 0.05 * math.sin(time.time())
        self.pitch_gimbal = 0.1 * math.sin(time.time() * 0.5)
        self.yaw_gimbal = 0.2 * math.sin(time.time() * 0.3)
        self.pitch_chassis = 0.0
        self.yaw_chassis = 0.3 * math.sin(time.time() * 0.2)
        
        # 如果有活动场景，调用场景函数
        if self.active_scenario:
            self.active_scenario()
        
        # 减少剩余时间
        if self.game_status >= 64 and self.remaining_time > 0:
            self.remaining_time -= 1
    
    def print_status(self):
        """打印当前状态"""
        # 比赛状态文本
        game_states = {
            0: "未开始",
            16: "准备阶段",
            32: "裁判自检",
            48: "倒计时",
            64: "比赛中",
            80: "结算中"
        }
        game_state_txt = game_states.get(self.game_status, "未知")
        
        # 场景名称
        scenario_name = "无"
        for key, func in self.scenarios.items():
            if func == self.active_scenario:
                scenario_name = f"{key}:{func.__doc__.split('：')[1] if '：' in func.__doc__ else func.__doc__}"
                break
        
        status_text = (
            f"\r状态: 血量={self.blood}/400, 比赛={game_state_txt}, "
            f"撤退={'ON' if self.is_retreating else 'OFF'}, "
            f"无人机={'ON' if self.is_drone_avoiding else 'OFF'}, "
            f"场景={scenario_name}{' [暂停]' if self.paused else ''}{' [记录中]' if self.is_logging else ''}"
        )
        print(status_text)
        sys.stdout.flush()
    
    def save_log_to_file(self):
        """保存日志到文件"""
        if not self.data_log:
            print("\n没有数据需要保存")
            return False
            
        try:
            filename = f"serial_data_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, 'w') as f:
                # 写入CSV表头
                f.write("时间戳,方向,原始数据,模式,状态,血量,撤退,防无人机,pitch_diff,yaw_diff,距离,开火建议,vx,vy,vw\n")
                
                # 写入数据行
                for entry in self.data_log:
                    if entry['direction'] == 'TX':
                        f.write(f"{entry['timestamp']},TX,{entry['raw_data']},{entry['mode']},{entry['status']}," + 
                                f"{entry['blood']},{entry['retreating']},{entry['drone_avoiding']},,,,,,,,\n")
                    else:  # RX
                        f.write(f"{entry['timestamp']},RX,{entry['raw_data']},,,,,,{entry['pitch_diff']:.2f}," + 
                                f"{entry['yaw_diff']:.2f},{entry['distance']:.2f},{entry['fire_advice']}," +
                                f"{entry['vx']:.2f},{entry['vy']:.2f},{entry['vw']:.2f}\n")
            
            print(f"\n已保存数据日志到 {filename}")
            return True
        except Exception as e:
            print(f"\n保存数据日志失败: {e}")
            return False
    
    def run(self):
        """运行测试客户端"""
        if not self.open_serial():
            print(f"无法打开串口: {self.port}")
            return
        
        print(f"串口已打开: {self.port}")
        print("输入命令:")
        print("  1-5: 选择测试场景")
        print("  p: 暂停/继续")
        print("  r: 重置数据")
        print("  d: 显示/隐藏原始数据")
        print("  l: 开始/停止记录数据")
        print("  s: 保存记录的数据到文件")
        print("  t: 发送测试数据包")
        print("  a: 读取所有可用数据")
        print("  i: 显示数据结构信息")
        print("  z: 显示串口统计信息")
        print("  q: 退出")
        
        self.running = True
        
        # 启动后台线程处理串口通信
        def communication_thread():
            last_update = time.time()
            last_status = time.time() - 3.0  # 确保立即打印状态
            
            while self.running:
                current_time = time.time()
                
                # 固定频率更新和发送数据
                if current_time - last_update >= 0.05:  # 20Hz
                    self.update_data()
                    self.send_data()
                    last_update = current_time
                
                # 接收数据
                if self.receive_data():
                    # 如果接收到数据，则立即打印状态
                    if not self.show_raw_data:  # 如果不显示原始数据才打印状态
                        self.print_status()
                    last_status = current_time
                
                # 定期打印状态 (每2秒)
                if current_time - last_status >= 2.0 and not self.show_raw_data:
                    self.print_status()
                    last_status = current_time
                
                # 短暂睡眠，避免CPU占用过高
                time.sleep(0.01)
        
        # 启动通信线程
        thread = threading.Thread(target=communication_thread, daemon=True)
        thread.start()
        
        # 主线程处理用户输入
        try:
            while self.running:
                cmd = input("")  # 在这里等待用户输入
                
                if cmd == 'q':
                    print("正在退出...")
                    self.running = False
                elif cmd == 'p':
                    self.paused = not self.paused
                    status = "暂停" if self.paused else "继续"
                    print(f"\n测试已{status}")
                elif cmd == 'r':
                    self.reset_data()
                    print("\n已重置所有数据")
                elif cmd == 'd':
                    self.show_raw_data = not self.show_raw_data
                    status = "显示" if self.show_raw_data else "隐藏"
                    print(f"\n已{status}原始数据")
                elif cmd == 'l':
                    self.is_logging = not self.is_logging
                    if self.is_logging:
                        self.data_log = []  # 清空之前的数据
                        print("\n开始记录数据...")
                    else:
                        print(f"\n停止记录数据，共记录 {len(self.data_log)} 条")
                elif cmd == 's':
                    self.save_log_to_file()
                elif cmd == 't':
                    print("\n发送测试数据包...")
                    self.send_test_data()
                elif cmd == 'a':
                    self.read_all_data()
                elif cmd == 'i':
                    self.print_data_structure()
                elif cmd == 'z':
                    self.print_serial_stats()
                elif cmd in self.scenarios:
                    self.active_scenario = self.scenarios[cmd]
                    print(f"\n激活场景: {self.active_scenario.__doc__}")
                    # 立即应用场景设置
                    self.active_scenario()
                else:
                    print(f"\n未知命令: {cmd}")
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as e:
            print(f"\n发生错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            thread.join(1.0)  # 等待通信线程结束
            self.close_serial()
            print("\n测试客户端已关闭")
    
    # === 场景函数 ===
    
    def scenario_normal_match(self):
        """场景1：正常比赛"""
        # 设置比赛状态为进行中
        self.game_status = 64  # 0x40，比赛中
        
        # 稳定血量
        if self.blood < 400 and random.random() < 0.05:
            self.blood += 1
    
    def scenario_low_health(self):
        """场景2：低血量测试"""
        # 设置比赛状态为进行中
        self.game_status = 64  # 0x40，比赛中
        
        # 逐渐减少血量
        if self.blood > 0 and random.random() < 0.2:
            self.blood -= 3
            if self.blood < 0:
                self.blood = 0
    
    def scenario_emergency_retreat(self):
        """场景3：紧急撤退"""
        # 设置比赛状态为进行中
        self.game_status = 64  # 0x40，比赛中
        
        # 设置撤退标志
        self.is_retreating = 1
        
        # 模拟血量波动
        if random.random() < 0.1:
            delta = random.randint(-10, 5)
            self.blood = max(0, min(400, self.blood + delta))
    
    def scenario_drone_attack(self):
        """场景4：无人机攻击"""
        # 设置比赛状态为进行中
        self.game_status = 64  # 0x40，比赛中
        
        # 设置无人机标志
        self.is_drone_avoiding = 1
        
        # 模拟无人机攻击导致的血量损失
        if random.random() < 0.15:
            self.blood = max(0, self.blood - random.randint(5, 15))
    
    def scenario_match_phases(self):
        """场景5：比赛阶段转换"""
        # 循环比赛各阶段
        phase_time = int(time.time()) % 50  # 50秒一个完整周期
        
        if phase_time < 10:  # 0-10秒：比赛未开始
            self.game_status = 0
            self.blood = 400
            self.remaining_time = 180
            
        elif phase_time < 15:  # 10-15秒：准备阶段
            self.game_status = 16  # 0x10
            self.blood = 400
            
        elif phase_time < 20:  # 15-20秒：裁判系统自检
            self.game_status = 32  # 0x20
            self.blood = 400
            
        elif phase_time < 25:  # 20-25秒：倒计时
            self.game_status = 48  # 0x30
            self.blood = 400
            
        else:  # 25-50秒：比赛中
            self.game_status = 64  # 0x40
            if phase_time > 45:  # 最后5秒模拟结算
                self.game_status = 80  # 0x50
    
    def reset_data(self):
        """重置数据到初始状态"""
        self.active_scenario = None
        self.decator_mode = 2
        self.roll = 0.0
        self.pitch_gimbal = 0.0
        self.yaw_gimbal = 0.0
        self.pitch_chassis = 0.0
        self.yaw_chassis = 0.0
        self.game_status = 0
        self.remaining_time = 180
        self.blood = 400
        self.is_free_rebirth = 0
        self.is_retreating = 0
        self.is_drone_avoiding = 0
        self.is_outpost_attacking = 0

# 主函数
def main():
    parser = argparse.ArgumentParser(description='哨兵机器人虚拟串口测试客户端')
    parser.add_argument('--port', type=str, default='/dev/ttyVIRTUAL2', 
                        help='虚拟串口设备路径')
    parser.add_argument('--baud', type=int, default=115200,
                        help='波特率')
    args = parser.parse_args()
    
    # 创建并运行测试客户端
    client = SimpleTestClient(args.port, args.baud)
    try:
        client.run()
    except Exception as e:
        print(f"客户端运行错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()