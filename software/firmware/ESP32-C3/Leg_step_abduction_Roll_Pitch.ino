#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "SparkFun_BNO080_Arduino_Library.h" // Библиотека SparkFun

BNO080 myIMU;

// --- НАСТРОЙКИ ПИНОВ И CAN ДЛЯ ESP32-C3 SUPER MINI ---
#define CAN_TX_PIN   GPIO_NUM_6
#define CAN_RX_PIN   GPIO_NUM_7
#define IMU_INT_PIN  GPIO_NUM_3
#define IMU_RST_PIN  GPIO_NUM_2

// --- НАСТРОЙКИ ГЕОМЕТРИИ ПЯТИЗВЕННИКА (aaedmusa) ---
const float E = 0.065f;  // Расстояние между осями моторов (65 мм)
const float C = 0.100f;  // Длина ведущего плеча (100 мм)
const float D = 0.150f;  // Длина ведомого плеча (150 мм)
const float H = 0.054f;  // Смещение моторов вниз (54 мм)

const float X_A = -E / 2.0f;
const float Z_A = -H;
const float X_B = E / 2.0f;
const float Z_B = -H;
const float C2 = C * C;
const float D2 = D * D;

// --- НАСТРОЙКИ РЕДУКТОРА И CAN ID ---
const float GEAR_RATIO = 10.0f; // Кабестатический редуктор 10:1
const uint8_t NODE_ID_A = 0;    // Левое плечо
const uint8_t NODE_ID_B = 1;    // Правое плечо
const uint8_t NODE_ID_ABAD = 2; // Абдукция (3-й мотор)

const uint16_t CMD_SET_AXIS_STATE = 0x007;
const uint16_t CMD_SET_INPUT_POS  = 0x00C;
const uint16_t CMD_HEARTBEAT      = 0x001;
const uint32_t AXIS_STATE_CLOSED_LOOP = 8;

// Тайминги и шаг
unsigned long lastLoopTime = 0;
const int LOOP_PERIOD_MS = 5; // Частота обновления 200 Гц
int frame_counter = 0;
const int NUM_STEPS = 200;    // 200 точек = 1 секунда на шаг

// --- ДИНАМИЧЕСКИЕ ПАРАМЕТРЫ ТРАЕКТОРИИ (УПРАВЛЯЮТСЯ С ДЖОЙСТИКА) ---
float z_height = -0.19f;       // Высота подвеса корпуса (дефолт -19 см)
float stride = 0.09f;          // Длина шага (дефолт 9 см)
float step_clearance = 0.045f; // Высота подъема стопы (дефолт 4.5 см)
float abad_angle = 0.0f;       // Целевой угол абдукции с джойстика

// Углы из IMU сопроцессора BNO086 (в радианах)
float body_roll = 0.0f;        // Текущий крэн (завал влево/вправо)
float body_pitch = 0.0f;       // Текущий тангаж (завал вперед/назад)

float trajectory_x[NUM_STEPS];
float trajectory_z[NUM_STEPS];

enum RobotState { INIT_SMOOTH_START, RUN_GAIT };
RobotState currentState = INIT_SMOOTH_START;
unsigned long initStartTime = 0;
const unsigned long INIT_DURATION_MS = 1000; // Плавный спуск длится 1 секунду

float start_x = 0.0f;
float start_z = -0.165f; // Стартовый ручной горизонт плеч

// Переменные плавного старта абдукции
float odrive_start_turns_ABAD = 0.0f;
bool got_pos_ABAD = false;

struct JointAngles {
    float theta_a;
    float theta_b;
    bool valid;
};
// Обратная кинематика (Математика широкого РОМБА без перекрещивания звеньев)
JointAngles solveInverseKinematics(float x, float z) {
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

    memcpy(message.data, &target_turns, sizeof(float));
    float zero_ff = 0.0f;
    memcpy(message.data + 4, &zero_ff, sizeof(float));

    twai_transmit(&message, pdMS_TO_TICKS(10));
}

// Генератор точек кубической кривой Безье
void bezier_curve(float p0_x, float p0_z, float p1_x, float p1_z, float p2_x, float p2_z, float p3_x, float p3_z, int steps, int start_idx) {
    for (int i = 0; i < steps; i++) {
        float t = (float)i / (steps - 1);
        float mt = 1.0f - t;
        int idx = start_idx + i;
        trajectory_x[idx] = mt*mt*mt*p0_x + 3*mt*mt*t*p1_x + 3*mt*t*t*p2_x + t*t*t*p3_x;
        trajectory_z[idx] = mt*mt*mt*p0_z + 3*mt*mt*t*p1_z + 3*mt*t*t*p2_z + t*t*t*p3_z;
    }
}

// Пересчет траектории шага
void updateTrajectory() {
    int half = NUM_STEPS / 2;

    // 1. Фаза опоры (Stance)
    for (int i = 0; i < half; i++) {
        float t = (float)i / (half - 1);
        trajectory_x[i] = (stride / 2.0f) - t * stride;
        trajectory_z[i] = z_height;
    }

    // 2. Фаза переноса (Swing) по Безье
    bezier_curve(
        -stride / 2.0f, z_height,                  // p0
        -stride / 2.0f, z_height + step_clearance, // p1
        stride / 2.0f,  z_height + step_clearance, // p2
        stride / 2.0f,  z_height,                  // p3
        half, half
    );
}
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
            case 'r': abad_angle += 0.0349f; if(abad_angle > 0.523f) abad_angle = 0.523f; changed = true; break;
            case 'f': abad_angle -= 0.0349f; if(abad_angle < -0.523f) abad_angle = -0.523f; changed = true; break;
            case 'x': z_height = -0.19f; stride = 0.09f; step_clearance = 0.045f; abad_angle = 0.0f; changed = true; break;
        }
        if (changed) {
            updateTrajectory();
            Serial.print("[JOYSTICK] Высота(Z): "); Serial.print(z_height * 1000.0f, 0); Serial.print(" мм | ");
            Serial.print("Шаг: "); Serial.print(stride * 1000.0f, 0); Serial.print(" Наклон: "); Serial.println(abad_angle * 57.3f, 1);
        }
    }
}

void checkIncomingCAN() {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) {
        uint32_t node_id = rx_msg.identifier >> 5;
        uint32_t cmd_id = rx_msg.identifier & 0x01F;

        if (cmd_id == CMD_HEARTBEAT) {
            uint32_t axis_error; uint8_t current_state;
            memcpy(&axis_error, rx_msg.data, sizeof(uint32_t));
            current_state = rx_msg.data[0]; // СТРОГИЙ ИСПРАВЛЕННЫЙ ИНДЕКС МАССИВА ДЛЯ ТИПА uint8_t
            if (axis_error != 0) { Serial.print("[CAN ERROR] ODrive "); Serial.println(node_id); }
        }

        if (cmd_id == 0x009 && node_id == NODE_ID_ABAD && !got_pos_ABAD) {
            memcpy(&odrive_start_turns_ABAD, rx_msg.data, sizeof(float));
            got_pos_ABAD = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== ЗАПУСК ПОЛНОЙ 3D СТАБИЛИЗАЦИИ (ROLL + PITCH) - FIX АБДУКЦИИ ===");

    // Аппаратный жесткий сброс BNO086
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW); delay(50);
    digitalWrite(IMU_RST_PIN, HIGH); delay(150);
    pinMode(IMU_INT_PIN, INPUT_PULLUP);

    Wire.begin(8, 9); Wire.setClock(100000);
    if (myIMU.begin(0x4B, Wire, IMU_INT_PIN) == false) {
        Serial.println("[FAIL] Датчик BNO086 не обнаружен!");
        while(1) delay(10);
    }
    Wire.setClock(400000);
    myIMU.enableGameRotationVector(10); // Включаем IMU на 100 Гц

    updateTrajectory();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config); twai_start();

    // --- УМНОЕ ОЖИДАНИЕ ПИТАНИЯ МОТОРОВ ---
    Serial.println("[SYSTEM] Ожидание включения питания ODrive (Heartbeat)...");
    bool odrive_online = false;
    while (!odrive_online) {
        twai_message_t rx_msg;
        // Ждем пакеты. Если ODrive выключен, мы просто слушаем тишину (без ошибок Bus-Off)
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            uint32_t cmd_id = rx_msg.identifier & 0x01F;
            if (cmd_id == CMD_HEARTBEAT) {
                odrive_online = true;
                Serial.println("[SYSTEM] ODrive обнаружен в сети! Продолжаем загрузку.");
            }
        }
        // Заодно продолжаем слушать джойстик
        handleVirtualJoystick(); 
    }

    // Даем еще секунду драйверам на полную инициализацию
    unsigned long waitStart = millis();
    while(millis() - waitStart < 1000) { checkIncomingCAN(); delay(1); }
    if (!got_pos_ABAD) odrive_start_turns_ABAD = 0.0f;

    sendODriveState(NODE_ID_A, AXIS_STATE_CLOSED_LOOP); delay(20);
    sendODriveState(NODE_ID_B, AXIS_STATE_CLOSED_LOOP); delay(20);
    sendODriveState(NODE_ID_ABAD, AXIS_STATE_CLOSED_LOOP); delay(20);

    unsigned long lockStart = millis();
    while (millis() - lockStart < 200) {
        sendODrivePos(NODE_ID_A, 0.0f); sendODrivePos(NODE_ID_B, 0.0f);

        // Убрали сдвиг на 15 градусов. Фиксируем абдукцию строго в текущем начальном положении
        sendODrivePos(NODE_ID_ABAD, odrive_start_turns_ABAD);

        checkIncomingCAN(); delay(5);
    }
    initStartTime = millis();
}
void loop() {
    checkIncomingCAN();
    handleVirtualJoystick();

    // Считывание данных двухосевого завала из сопроцессора BNO086
    if (myIMU.dataAvailable() == true) {
        body_roll = myIMU.getRoll();   // Получаем крэн корпуса (влево/вправо)
        body_pitch = myIMU.getPitch(); // Получаем тангаж корпуса (вперед/назад)
    }

    unsigned long currentTime = millis();
    if (currentTime - lastLoopTime >= LOOP_PERIOD_MS) {
        lastLoopTime = currentTime;
        float target_x = 0; float target_z = 0;

        if (currentState == INIT_SMOOTH_START) {
            unsigned long elapsed = currentTime - initStartTime;
            if (elapsed < INIT_DURATION_MS) {
                float ratio = (float)elapsed / INIT_DURATION_MS;
                target_x = start_x + ratio * (0.0f - start_x);
                target_z = start_z + ratio * (z_height - start_z);

                // Удерживаем мотор абдукции в его начальном положении
                sendODrivePos(NODE_ID_ABAD, odrive_start_turns_ABAD);
            } else {
                currentState = RUN_GAIT; frame_counter = 0;
                Serial.println("[SYSTEM] Полная 3D-стабилизация активна. Запуск шага!");
                return;
            }
        }
        else if (currentState == RUN_GAIT) {
            target_x = trajectory_x[frame_counter];
            target_z = trajectory_z[frame_counter];
            frame_counter++;
            if (frame_counter >= NUM_STEPS) frame_counter = 0;
        }

        // Автоматическая стабилизация по оси Pitch (Плечевые моторы)
        if (currentState == RUN_GAIT) {
            // ПРЕДОХРАНИТЕЛЬ: Ограничиваем Pitch диапазоном от -10 до +10 градусов (-0.174 до +0.174 радиан)
            // Это спасет от бесконечности тангенса при сильных завалах и шумах датчика от других осей
            float safe_pitch = constrain(body_pitch, -0.174f, 0.174f);

            // Наклон вперед (положительный Pitch) должен сместить стопу назад для удержания баланса.
            // Смещение вычисляется через текущую высоту ноги: z * tan(pitch)
            float pitch_correction_x = target_z * tan(safe_pitch);

            // Корректируем координату X траектории перед подачей в инверсную кинематику
            target_x += pitch_correction_x;
        }

        // Вычисление инверсной кинематики пятизвенника с учетом скорректированного X
        JointAngles angles = solveInverseKinematics(target_x, target_z);
        if (angles.valid) {
            float angle_B = angles.theta_b;
            float angle_A = angles.theta_a;
            if (angle_A > 0.0f) angle_A -= 2.0f * PI;
            float normal_angle_A = angle_A - (-PI);

            float turns_A = (normal_angle_A / (2.0f * PI)) * GEAR_RATIO;
            float turns_B = (angle_B / (2.0f * PI)) * GEAR_RATIO;

            // Управление мотором абдукции
            if (currentState == RUN_GAIT) {
                // ПРЕДОХРАНИТЕЛЬ: Ограничиваем Roll диапазоном от -30 до +30 градусов (-0.523 до +0.523 радиан)
                float safe_roll = constrain(body_roll, -0.523f, 0.523f);

                // Целевой угол = ручное управление с джойстика МИНУС текущий завал корпуса
                float total_abad_angle = abad_angle - safe_roll;

                // Переводим радианы в обороты вала ODrive
                float dynamic_abad_turns = (total_abad_angle / (2.0f * PI)) * GEAR_RATIO;

                // Прибавляем к стартовой (нулевой) позиции
                float turns_ABAD = odrive_start_turns_ABAD + dynamic_abad_turns;

                sendODrivePos(NODE_ID_ABAD, turns_ABAD);
            }

            sendODrivePos(NODE_ID_A, turns_A);
            sendODrivePos(NODE_ID_B, turns_B);
        }
    }
}
