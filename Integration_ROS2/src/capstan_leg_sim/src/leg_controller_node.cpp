#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class LegControllerNode : public rclcpp::Node
{
public:
    LegControllerNode() : Node("leg_controller_node")
    {
        // Подписка на IMU (аналог BNO086)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&LegControllerNode::imuCallback, this, std::placeholders::_1));

        // Подписка на символьные команды (аналог Serial джойстика)
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/serial_cmd", 10, std::bind(&LegControllerNode::cmdCallback, this, std::placeholders::_1));

        // Паблишер команд для ros2_control (JointGroupPositionController)
        joint_pub_ = this->create_publisher<std_msgs::msg::Float64_multi_array>(
            "/position_controller/commands", 10);

        // Инициализация траектории
        updateTrajectory();

        // Таймер на 200 Гц (5 мс)
        timer_ = this->create_wall_timer(5ms, std::bind(&LegControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Leg Controller Node started. Publishing at 200Hz.");
    }

private:
    // Константы геометрии из ESP32
    const float E = 0.065f;
    const float C = 0.100f;
    const float D = 0.150f;
    const float H = 0.054f;
    const float X_A = -E / 2.0f;
    const float Z_A = -H;
    const float X_B = E / 2.0f;
    const float Z_B = -H;
    const float C2 = C * C;
    const float D2 = D * D;

    // В ROS 2 контроллер принимает радианы напрямую, 
    // поэтому GEAR_RATIO не применяется к финальным командам.
    // const float GEAR_RATIO = 10.0f; 

    static const int NUM_STEPS = 200;
    
    // Динамические параметры (с джойстика)
    float z_height = -0.19f;
    float stride = 0.09f;
    float step_clearance = 0.045f;
    float abad_angle = 0.0f;

    // Данные от IMU
    float body_roll = 0.0f;
    float body_pitch = 0.0f;

    float trajectory_x[NUM_STEPS];
    float trajectory_z[NUM_STEPS];
    int frame_counter = 0;

    // ROS 2 интерфейсы
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    struct JointAngles {
        float theta_a;
        float theta_b;
        bool valid;
    };

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Конвертируем кватернион в углы Эйлера (как отдает BNO086)
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        body_roll = static_cast<float>(roll);
        body_pitch = static_cast<float>(pitch);
    }

    void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data.empty()) return;
        char key = msg->data[0];
        bool changed = false;

        switch (key) {
            case 'w': z_height += 0.005f; if(z_height > -0.14f) z_height = -0.14f; changed = true; break;
            case 's': z_height -= 0.005f; if(z_height < -0.23f) z_height = -0.23f; changed = true; break;
            case 'd': stride += 0.010f; if(stride > 0.14f) stride = 0.14f; changed = true; break;
            case 'a': stride -= 0.010f; if(stride < -0.06f) stride = -0.06f; changed = true; break;
            case 'q': step_clearance += 0.005f; if(step_clearance > 0.07f) step_clearance = 0.07f; changed = true; break;
            case 'e': step_clearance -= 0.005f; if(step_clearance < 0.01f) step_clearance = 0.01f; changed = true; break;
            case 'r': abad_angle += 0.0349f; if(abad_angle > 0.523f) abad_angle = 0.523f; changed = true; break;
            case 'f': abad_angle -= 0.0349f; if(abad_angle < -0.523f) abad_angle = -0.523f; changed = true; break;
            case 'x': z_height = -0.19f; stride = 0.09f; step_clearance = 0.045f; abad_angle = 0.0f; changed = true; break;
        }

        if (changed) {
            updateTrajectory();
            RCLCPP_INFO(this->get_logger(), "[JOYSTICK] Z: %.1f mm | Stride: %.1f mm | Abad: %.1f deg", 
                z_height * 1000.0f, stride * 1000.0f, abad_angle * 57.3f);
        }
    }

    void bezier_curve(float p0_x, float p0_z, float p1_x, float p1_z, float p2_x, float p2_z, float p3_x, float p3_z, int steps, int start_idx) 
    {
        for (int i = 0; i < steps; i++) {
            float t = (float)i / (steps - 1);
            float mt = 1.0f - t;
            int idx = start_idx + i;
            trajectory_x[idx] = mt*mt*mt*p0_x + 3*mt*mt*t*p1_x + 3*mt*t*t*p2_x + t*t*t*p3_x;
            trajectory_z[idx] = mt*mt*mt*p0_z + 3*mt*mt*t*p1_z + 3*mt*t*t*p2_z + t*t*t*p3_z;
        }
    }

    void updateTrajectory() 
    {
        int half = NUM_STEPS / 2;
        // Фаза опоры
        for (int i = 0; i < half; i++) {
            float t = (float)i / (half - 1);
            trajectory_x[i] = (stride / 2.0f) - t * stride;
            trajectory_z[i] = z_height;
        }
        // Фаза переноса
        bezier_curve(
            -stride / 2.0f, z_height,                  
            -stride / 2.0f, z_height + step_clearance, 
            stride / 2.0f,  z_height + step_clearance, 
            stride / 2.0f,  z_height,                  
            half, half
        );
    }

    JointAngles solveInverseKinematics(float x, float z) 
    {
        JointAngles angles;
        angles.valid = false;

        float dxA = x - X_A; float dzA = z - Z_A;
        float L_A = std::hypot(dxA, dzA);
        if (L_A > (C + D) || L_A < std::abs(C - D)) return angles;
        float gamma_A = std::atan2(dzA, dxA);
        float alpha_A = std::acos((C2 + L_A*L_A - D2) / (2.0f * C * L_A));

        float dxB = x - X_B; float dzB = z - Z_B;
        float L_B = std::hypot(dxB, dzB);
        if (L_B > (C + D) || L_B < std::abs(C - D)) return angles;
        float gamma_B = std::atan2(dzB, dxB);
        float alpha_B = std::acos((C2 + L_B*L_B - D2) / (2.0f * C * L_B));

        // Разводим колени строго наружу
        angles.theta_a = gamma_A - alpha_A;
        angles.theta_b = gamma_B + alpha_B;
        angles.valid = true;
        return angles;
    }

    void controlLoop()
    {
        float target_x = trajectory_x[frame_counter];
        float target_z = trajectory_z[frame_counter];
        
        frame_counter++;
        if (frame_counter >= NUM_STEPS) frame_counter = 0;

        // ПРЕДОХРАНИТЕЛЬ: Ограничиваем Pitch
        float safe_pitch = std::clamp(body_pitch, -0.174f, 0.174f);
        float pitch_correction_x = target_z * std::tan(safe_pitch);
        target_x += pitch_correction_x;

        JointAngles angles = solveInverseKinematics(target_x, target_z);
        if (angles.valid) {
            float angle_B = angles.theta_b;
            float angle_A = angles.theta_a;
            if (angle_A > 0.0f) angle_A -= 2.0f * M_PI;
            float normal_angle_A = angle_A - (-M_PI);

            // ПРЕДОХРАНИТЕЛЬ: Ограничиваем Roll
            float safe_roll = std::clamp(body_roll, -0.523f, 0.523f);
            float total_abad_angle = abad_angle - safe_roll;

            // Формирование сообщения для ros2_control (требует радианы!)
            // Порядок соответствует массиву из controllers.yaml: [joint_abad, joint_a, joint_b]
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.push_back(total_abad_angle);
            cmd_msg.data.push_back(normal_angle_A);
            cmd_msg.data.push_back(angle_B);
            
            joint_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegControllerNode>());
    rclcpp::shutdown();
    return 0;
}