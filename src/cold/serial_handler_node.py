#!/usr/bin/env python3

import rospy
from pymodbus.client import ModbusSerialClient as ModbusClient
from motor_control_pkg.srv import ModbusWrite, ModbusWriteResponse  # カスタムサービスのインポート
import serial
import threading

class SerialHandlerNode:
    def __init__(self):
        # Modbus Client の初期化
        self.client = ModbusClient(
            method='rtu',
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=3,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE
        )
        self.lock = threading.Lock()  # ロックを使ってアクセスを制御

        if not self.client.connect():
            rospy.logerr("Failed to connect to Modbus server")
            exit(1)

        # サービスの定義
        self.service = rospy.Service('modbus_write', ModbusWrite, self.handle_write_request)

    def handle_write_request(self, req):
        with self.lock:
            try:
                # リクエストの内容を取得
                address = req.address
                data = req.data
                slave_id = req.slave_id

                result = self.client.write_registers(address=address, values=data, unit=slave_id)

                # レスポンスの生成
                if result.isError():
                    return ModbusWriteResponse(success=False, message=str(result))
                else:
                    return ModbusWriteResponse(success=True, message="Write successful")
            except Exception as e:
                rospy.logerr(f"Modbus write failed: {e}")
                return ModbusWriteResponse(success=False, message=str(e))

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('serial_handler_node')
    node = SerialHandlerNode()
    node.run()
