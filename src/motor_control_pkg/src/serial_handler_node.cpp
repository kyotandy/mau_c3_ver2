#include <ros/ros.h>
#include <motor_control_pkg/ModbusWrite.h>
#include <modbus/modbus.h>
#include <mutex>
#include <vector>
#include <cstdint>

class SerialHandlerNode {
public:
    SerialHandlerNode() : last_modbus_time(ros::Time::now()) {
        // Modbus client の初期化
        ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'E', 8, 1); // パリティは Even, ストップビットは 1
        modbus_set_response_timeout(ctx, 1, 0); // タイムアウト1秒

        if (ctx == NULL) {
            ROS_ERROR("Unable to create the Modbus context");
            exit(1);
        }

        if (modbus_connect(ctx) == -1) {
            ROS_ERROR("Failed to connect to the Modbus server");
            modbus_free(ctx);
            exit(1);
        }

        // サブスクライバーを作成
        sub = nh.subscribe("modbus_request", 1000, &SerialHandlerNode::handleModbusRequest, this);
    }

    void handleModbusRequest(const motor_control_pkg::ModbusWrite::ConstPtr& req) {
        std::unique_lock<std::mutex> lock(modbus_mutex);

        // Modbus slave IDの設定
        int rc = modbus_set_slave(ctx, req->slave_id);
        if (rc == -1) {
            ROS_ERROR("Failed to set Modbus slave ID");
            return;
        }

        // リクエストされたデータを書き込む
        std::vector<uint16_t> data(req->data.size());
        for (size_t i = 0; i < req->data.size(); ++i) {
            data[i] = static_cast<uint16_t>(req->data[i]);  // int から uint16_t へキャスト
        }

        // Modbusレジスタに書き込み
        rc = modbus_write_registers(ctx, req->address, data.size(), data.data());
        if (rc == -1) {
            ROS_ERROR("Modbus write failed: %s", modbus_strerror(errno));
            return;
        }

        ROS_INFO("Modbus write successful");
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
    ros::Subscriber sub;
    modbus_t *ctx; // Modbusコンテキスト
    std::mutex modbus_mutex; // スレッドセーフのためのミューテックス
    ros::Time last_modbus_time; // 最後にModbus通信を行った時間
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_handler_node");
    SerialHandlerNode node;
    node.run();
    return 0;
}
