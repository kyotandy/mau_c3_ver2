#include <ros/ros.h>
#include <motor_control_pkg/ModbusWrite.h> // カスタムサービスのインポート
#include <modbus/modbus.h>
#include <mutex>
#include <vector>
#include <cstdint>

class SerialHandlerNode {
public:
    SerialHandlerNode() {
        // Modbus client の初期化
        ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'E', 8, 1); // パリティは Even, ストップビットは 1
        modbus_set_response_timeout(ctx, 1, 0);

        if (ctx == NULL) {
            ROS_ERROR("Unable to create the Modbus context");
            exit(1);
        }

        if (modbus_connect(ctx) == -1) {
            ROS_ERROR("Failed to connect to the Modbus server");
            modbus_free(ctx);
            exit(1);
        }

        // サービスを作成
        service = nh.advertiseService("modbus_write", &SerialHandlerNode::handleWriteRequest, this);
    }

    bool handleWriteRequest(motor_control_pkg::ModbusWrite::Request &req,
                            motor_control_pkg::ModbusWrite::Response &res) {
        std::lock_guard<std::mutex> lock(modbus_mutex);

        int rc = modbus_set_slave(ctx, req.slave_id);
        if (rc == -1) {
            ROS_ERROR("Failed to set Modbus slave ID");
            res.success = false;
            res.message = "Failed to set slave ID";
            return false;
        }

        // リクエストされたデータを書き込む
        std::vector<uint16_t> data(req.data.size());
        for (size_t i = 0; i < req.data.size(); ++i) {
            data[i] = static_cast<uint16_t>(req.data[i]);  // int から uint16_t へキャスト
        }
    
        rc = modbus_write_registers(ctx, req.address, data.size(), data.data());
        if (rc == -1) {
            ROS_ERROR("Modbus write failed: %s", modbus_strerror(errno));
            res.success = false;
            res.message = modbus_strerror(errno);
            return false;
        }

        res.success = true;
        res.message = "Write successful";
        return true;
    }

    void run() {
        ros::spin();
    }

    ~SerialHandlerNode() {
        modbus_close(ctx);
        modbus_free(ctx);
    }

private:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    modbus_t *ctx; // Modbusコンテキスト
    std::mutex modbus_mutex; // スレッドセーフのためのミューテックス
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_handler_node");
    SerialHandlerNode node;
    node.run();
    return 0;
}
