#include <Arduino.h>
#include "driver/twai.h"

// --- НАСТРОЙКИ ГЕОМЕТРИИ (aaedmusa) ---
const float E = 0.065f;  const float C = 0.100f;  const float D = 0.150f;  const float H = 0.054f;
const float X_A = -E / 2.0f; const float Z_A = -H; const float X_B = E / 2.0f; const float Z_B = -H;
const float C2 = C * C; const float D2 = D * D;

// --- НАСТРОЙКИ РЕДУКТОРА И CAN ODRIVE ---
const float GEAR_RATIO = 10.0f;
const uint8_t NODE_ID_A = 0;    // Левое плечо
const uint8_t NODE_ID_B = 1;    // Правое плечо
const uint8_t NODE_ID_ABAD = 2; // НОВОЕ: Абдукция (3-й мотор)

const uint16_t CMD_SET_AXIS_STATE = 0x007;
const uint16_t CMD_SET_INPUT_POS  = 0x00C;
const uint16_t CMD_HEARTBEAT      = 0x001;
const uint32_t AXIS_STATE_CLOSED_LOOP = 8;

#define CAN_TX_PIN GPIO_NUM_6
#define CAN_RX_PIN GPIO_NUM_7

// Тайминги и шаг
unsigned long lastLoopTime = 0;
const int LOOP_PERIOD_MS = 5;
int frame_counter = 0;
const int NUM_STEPS = 200;

// --- ДИНАМИЧЕСКИЕ ПАРАМЕТРЫ ТРАЕКТОРИИ ---
float z_height = -0.19f;
float stride = 0.09f;
float step_clearance = 0.045f;
float abad_angle = 0.0f;       // НОВОЕ: Угол наклона абдукции в радианах (дефолт 0.0)

float trajectory_x[NUM_STEPS];
float trajectory_z[NUM_STEPS];

enum RobotState { INIT_SMOOTH_START, RUN_GAIT };
RobotState currentState = INIT_SMOOTH_START;
unsigned long initStartTime = 0;
const unsigned long INIT_DURATION_MS = 1000;

float start_x = 0.0f;
float start_z = -0.165f;

struct JointAngles { float theta_a; float theta_b; bool valid; };

JointAngles solveInverseKinematics(float x, float z) {
    JointAngles angles; angles.valid = false;
    float dxA = x - X_A; float dzA = z - Z_A; float L_A = std::hypot(dxA, dzA);
    if (L_A > (C + D) || L_A < std::abs(C - D)) return angles;
    float gamma_A = std::atan2(dzA, dxA);
    float alpha_A = std::acos((C2 + L_A*L_A - D2) / (2.0f * C * L_A));

    float dxB = x - X_B; float dzB = z - Z_B; float L_B = std::hypot(dxB, dzB);
    if (L_B > (C + D) || L_B < std::abs(C - D)) return angles;
    float gamma_B = std::atan2(dzB, dxB);
    float alpha_B = std::acos((C2 + L_B*L_B - D2) / (2.0f * C * L_B));

    angles.theta_a = gamma_A - alpha_A;
    angles.theta_b = gamma_B + alpha_B;
    angles.valid = true;
    return angles;
}

void sendODriveState(uint8_t nodeId, uint32_t requested_state) {
    twai_message_t message;
    message.identifier = (nodeId << 5) | CMD_SET_AXIS_STATE;
    message.extd = 0; message.rtr = 0; message.data_length_code = 4;
    memcpy(message.data, &requested_state, sizeof(uint32_t));
    twai_transmit(&message, pdMS_TO_TICKS(10));
}

void sendODrivePos(uint8_t nodeId, float target_turns) {
    twai_message_t message;
    message.identifier = (nodeId << 5) | CMD_SET_INPUT_POS;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 8;

    // Безопасное побайтовое разделение float (4 байта)
    uint8_t* float_ptr = (uint8_t*)&target_turns;
    message.data[0] = float_ptr[0];
    message.data[1] = float_ptr[1];
    message.data[2] = float_ptr[2];
    message.data[3] = float_ptr[3];

    // Зануляем оставшиеся 4 байта (Velocity Feed-forward и Torque Feed-forward)
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;

    twai_transmit(&message, pdMS_TO_TICKS(10));
}

void bezier_curve(float p0_x, float p0_z, float p1_x, float p1_z, float p2_x, float p2_z, float p3_x, float p3_z, int steps, int start_idx) {
    for (int i = 0; i < steps; i++) {
        float t = (float)i / (steps - 1); float mt = 1.0f - t; int idx = start_idx + i;
        trajectory_x[idx] = mt*mt*mt*p0_x + 3*mt*mt*t*p1_x + 3*mt*t*t*p2_x + t*t*t*p3_x;
        trajectory_z[idx] = mt*mt*mt*p0_z + 3*mt*mt*t*p1_z + 3*mt*t*t*p2_z + t*t*t*p3_z;
    }
}

void updateTrajectory() {
    int half = NUM_STEPS / 2;
    for (int i = 0; i < half; i++) {
        float t = (float)i / (half - 1);
        trajectory_x[i] = (stride / 2.0f) - t * stride;
        trajectory_z[i] = z_height;
    }
    bezier_curve(-stride / 2.0f, z_height, -stride / 2.0f, z_height + step_clearance, stride / 2.0f, z_height + step_clearance, stride / 2.0f, z_height, half, half);
}

// НОВОЕ: Обработка клавиш R и F для управления наклоном абдукции
void handleVirtualJoystick() {
    if (Serial.available() > 0) {
        char key = Serial.read();
        bool changed = false;

        switch (key) {
            case 'w': z_height += 0.005f; if(z_height > -0.14f) z_height = -0.14f; changed = true; break;
            case 's': z_height -= 0.005f; if(z_height < -0.23f) z_height = -0.23f; changed = true; break;
            case 'd': stride += 0.010f; if(stride > 0.14f) stride = 0.14f; changed = true; break;
            case 'a': stride -= 0.010f; if(stride < -0.06f) stride = -0.06f; changed = true; break;
            case 'q': step_clearance += 0.005f; if(step_clearance > 0.07f) step_clearance = 0.07f; changed = true; break;
            case 'e': step_clearance -= 0.005f; if(step_clearance < 0.01f) step_clearance = 0.01f; changed = true; break;

            // --- НОВОЕ: Управление Абдукцией (Наклон ноги вбок) ---
            case 'r': abad_angle += 0.0349f; if(abad_angle > 0.523f) abad_angle = 0.523f; changed = true; break;  // Наклон вправо (+2 градуса), лимит +30°
            case 'f': abad_angle -= 0.0349f; if(abad_angle < -0.523f) abad_angle = -0.523f; changed = true; break; // Наклон влево (-2 градуса), лимит -30°

            case 'x': z_height = -0.19f; stride = 0.09f; step_clearance = 0.045f; abad_angle = 0.0f; changed = true; break;
        }

        if (changed) {
            updateTrajectory();
            Serial.print("[JOYSTICK] Высота(Z): "); Serial.print(z_height * 1000.0f, 0); Serial.print(" мм | ");
            Serial.print("Шаг: "); Serial.print(stride * 1000.0f, 0); Serial.print(" мм | ");
            Serial.print("Абдукция(Abad): "); Serial.print(abad_angle * 57.295f, 1); Serial.println(" град");
        }
    }
}

void checkIncomingCAN() {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) {
        uint32_t node_id = rx_msg.identifier >> 5;
        uint32_t cmd_id = rx_msg.identifier & 0x01F;
        if (cmd_id == CMD_HEARTBEAT) {
            uint32_t axis_error;
            memcpy(&axis_error, rx_msg.data, sizeof(uint32_t));
            if (axis_error != 0) {
                Serial.print("[CAN ERROR] ODrive "); Serial.print(node_id);
                Serial.print(" | Код ошибки: "); Serial.println(axis_error);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== ПРОШИВКА 3D ХОДЬБЫ С АБДУКЦИЕЙ ===");
    Serial.println("Управление клавиатурой в Serial Monitor:");
    Serial.println("  W / S - Высота корпуса (Z)");
    Serial.println("  A / D - Длина шага / Направление");
    Serial.println("  Q / E - Высота поджатия ноги (Клиренс)");
    Serial.println("  R / F - Наклон ноги влево/вправо (Абдукция)");
    Serial.println("  X     - Сбросить всё в дефолт");
    Serial.println("----------------------------------------");

    updateTrajectory();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    delay(1000);
    // НОВОЕ: Переводим ВСЕ ТРИ драйвера в закрытый контур
    sendODriveState(NODE_ID_A, AXIS_STATE_CLOSED_LOOP); delay(20);
    sendODriveState(NODE_ID_B, AXIS_STATE_CLOSED_LOOP); delay(20);
    sendODriveState(NODE_ID_ABAD, AXIS_STATE_CLOSED_LOOP); delay(20);

    // Фиксация стартового горизонта для всех осей
    unsigned long lockStart = millis();
    while (millis() - lockStart < 200) {
        sendODrivePos(NODE_ID_A, 0.0f);
        sendODrivePos(NODE_ID_B, 0.0f);
        sendODrivePos(NODE_ID_ABAD, 0.0f); // Удерживаем абдукцию в нуле
        delay(5);
    }
    initStartTime = millis();
}

void loop() {
    checkIncomingCAN();
    handleVirtualJoystick();

    unsigned long currentTime = millis();
    if (currentTime - lastLoopTime >= LOOP_PERIOD_MS) {
        lastLoopTime = currentTime;
        float target_x = 0; float target_z = 0;

        if (currentState == INIT_SMOOTH_START) {
            unsigned long elapsed = currentTime - initStartTime;
            if (elapsed < INIT_DURATION_MS) {
                float ratio = (float)elapsed / INIT_DURATION_MS;
                float end_x = 0.0f;
                float end_z = z_height;
                target_x = start_x + ratio * (end_x - start_x);
                target_z = start_z + ratio * (end_z - start_z);
            } else {
                currentState = RUN_GAIT; frame_counter = 0;
                Serial.println("[SYSTEM] Запуск цикла шага!");
                return;
            }
        }
        else if (currentState == RUN_GAIT) {
            target_x = trajectory_x[frame_counter];
            target_z = trajectory_z[frame_counter];
            frame_counter++;
            if (frame_counter >= NUM_STEPS) frame_counter = 0;
        }

        // Вычисление инверсной кинематики для пятизвенника
        JointAngles angles = solveInverseKinematics(target_x, target_z);
        if (angles.valid) {
            float angle_B = angles.theta_b;
            float angle_A = angles.theta_a;
            if (angle_A > 0.0f) angle_A -= 2.0f * PI;
            float normal_angle_A = angle_A - (-PI);

            float turns_A = (normal_angle_A / (2.0f * PI)) * GEAR_RATIO;
            float turns_B = (angle_B / (2.0f * PI)) * GEAR_RATIO;

            // НОВОЕ: Перевод угла абдукции в обороты вала ODrive 2
            // Знак перед abad_angle определяет сторону наклона (если наклоняется зеркально, добавьте минус)
            float turns_ABAD = (abad_angle / (2.0f * PI)) * GEAR_RATIO;

            // Отправляем команды на ВСЕ ТРИ платы
            sendODrivePos(NODE_ID_A, turns_A);
            sendODrivePos(NODE_ID_B, turns_B);
            sendODrivePos(NODE_ID_ABAD, turns_ABAD); // Посылка на 3-й мотор
        }
    }
}
