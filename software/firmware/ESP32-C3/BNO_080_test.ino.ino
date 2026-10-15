#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

BNO080 myIMU;

// Жестко привязываем пины к вашей схеме под пайку
#define IMU_INT_PIN GPIO_NUM_3
#define IMU_RST_PIN GPIO_NUM_2

unsigned long lastPrintTime = 0;
const int PRINT_PERIOD_MS = 50; 

void setup() {
    Serial.begin(115200);
    delay(3000); 

    Serial.println("\n=============================================");
    Serial.println("=== АППАРАТНЫЙ ЖЕСТКИЙ ЗАПУСК BNO086 (I2C) ===");
    Serial.println("=============================================");

    // 1. АППАРАТНЫЙ СБРОС ПО ДАТАШИТУ HILLCREST
    Serial.println("[IMU] Выполняем жесткий аппаратный Reset...");
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW);   // Опускаем RST в землю (чип спит)
    delay(50);                        // Держим сброс 50 мс
    digitalWrite(IMU_RST_PIN, HIGH);  // Поднимаем RST в 3.3V (чип просыпается)
    delay(150);                       // Обязательная пауза 150 мс на загрузку ARM-ядра датчика

    // 2. НАСТРОЙКА ЛИНИИ ПРЕРЫВАНИЯ
    // Датчику BNO необходима подтяжка к питанию на линии INT, чтобы исключить наводки
    pinMode(IMU_INT_PIN, INPUT_PULLUP); 

    // 3. ЗАПУСК ШИНЫ I2C
    Wire.begin(8, 9);
    Wire.setClock(100000); // Временно снижаем скорость до стандартных 100 кГц для стабильности инициализации

    Serial.println("[IMU] Пробуем связаться с SHTP-протоколом по адресу 0x4B...");

    // Запускаем библиотеку с жестко заданным адресом и пином прерывания
    if (myIMU.begin(0x4B, Wire, IMU_INT_PIN) == false) {
        Serial.println("[FAIL] Протокол SHTP не ответил.");
        Serial.println("[SYSTEM] Пробуем сделать программный сброс внутри библиотеки...");
        
        // Попытка без прерывания, если линия INT аппаратно повреждена
        if (myIMU.begin(0x4B, Wire) == false) {
            Serial.println("[CRITICAL FAIL] Датчик полностью заблокирован. Проверьте пайку!");
            
            // Запускаем бесконечный сканер I2C прямо здесь, чтобы вы видели, не отвалился ли чип
            while(1) {
                Wire.beginTransmission(0x4B);
                if (Wire.endTransmission() == 0) {
                    Serial.println("[SCANNER] Чип физически на связи (0x4B), но прошивка BNO зависла.");
                } else {
                    Serial.println("[SCANNER] Внимание! Чип полностью ИСЧЕЗ с шины I2C!");
                }
                delay(1000);
            }
        }
    }

    Serial.println("[OK] БИНГО! Датчик BNO086 успешно прошел инициализацию пакетов!");
    
    // Возвращаем шину I2C на быструю скорость 400 кГц после успешного старта
    Wire.setClock(400000); 

    // Включаем выдачу Game Rotation Vector (10 мс = 100 Гц)
    myIMU.enableGameRotationVector(10); 
    Serial.println("[SYSTEM] Графики запущены. Откройте Tools -> Serial Plotter.");
    Serial.println("-------------------------------------------------------------");
}

void loop() {
    if (myIMU.dataAvailable() == true) {
        unsigned long currentTime = millis();
        if (currentTime - lastPrintTime >= PRINT_PERIOD_MS) {
            lastPrintTime = currentTime; 

            // Получаем углы и переводим радианы в градусы
            float roll  = myIMU.getRoll() * 57.29577951f;   
            float pitch = myIMU.getPitch() * 57.29577951f;  

            Serial.print("Roll_Крен:");
            Serial.print(roll, 2);
            Serial.print(",");
            Serial.print("Pitch_Тангаж:");
            Serial.println(pitch, 2);
        }
    }
}
