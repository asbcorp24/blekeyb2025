#include <BleKeyboard.h>                // Подключаем библиотеку для эмуляции BLE-клавиатуры  
#include <BMI160Gen.h>                  // Подключаем библиотеку для работы с акселерометром BMI160/323  
#include <SPI.h>                        // Подключаем библиотеку для работы с интерфейсом SPI  
#include <NimBLEDevice.h>              // Подключаем базовую библиотеку NimBLE для BLE-стека  
#include "freertos/FreeRTOS.h"          // Подключаем заголовок ядра FreeRTOS  
#include "freertos/task.h"             // Подключаем API для создания и управления задачами FreeRTOS  

// ===== Пины подключения =====  
#define HSPI_CLK        14             // Номер пина CLK для шины HSPI (датчик BMI323)  
#define HSPI_SDI        12             // Номер пина MOSI (SDI) для шины HSPI (датчик BMI323)  
#define HSPI_SDO        13             // Номер пина MISO (SDO) для шины HSPI (датчик BMI323)  
#define HSPI_CS         25             // Номер пина CS (Chip Select) для шины HSPI (датчик BMI323)  
#define INT1_PIN        26             // Номер пина для входа прерывания INT1 от датчика BMI323  
#define INT2_PIN        27             // Номер пина для входа прерывания INT2 от датчика BMI323  

#define CHARGE_LED_PIN  22             // Номер пина для светодиода индикатора зарядки  
#define POWER_LED_PIN   23             // Номер пина для светодиода индикатора питания  
#define STATUS_LED_PIN  21             // Номер пина для светодиода индикатора статуса (BLE/ошибка)  

#define BATTERY_PIN     34             // Номер пина ADC для измерения напряжения батареи  

#define POWER_BTN_PIN   19             // Номер пина для кнопки питания  
#define LEFT_BTN_PIN    18             // Номер пина для кнопки «налево»  
#define RIGHT_BTN_PIN   17             // Номер пина для кнопки «направо»  
#define SIT_BTN_PIN     16             // Номер пина для кнопки «сесть»  
#define STAND_BTN_PIN   4              // Номер пина для кнопки «встать/идти»  
#define READY_BTN_PIN   5              // Номер пина для кнопки «стоп/сброс»  

// ===== FSM состояний =====  
enum State { OFF, ON, CHARGING, ERROR };          // Определяем перечисление режимов работы системы  
volatile State currentState = OFF;                // Объявляем глобальную переменную для текущего состояния, начинаем с OFF  

// ===== BLE-клавиатура =====  
BleKeyboard bleKeyboard("REM-Control",           // Создаём объект BLE-клавиатуры с именем устройства  
                        "REM-Manufacturer",      // Указываем производителя  
                        100);                   // Максимальное число уведомлений (battery level, etc.)  

// ===== Показатель уровня батареи =====  
volatile float batteryPct = 0.0f;                 // Глобальная переменная для хранения процента заряда батареи  

// ===== Мигалки для светодиодов =====  
struct Blinker {                                  // Определяем структуру для управления миганием светодиодов  
  uint8_t pin;                                    //  — пин светодиода  
  uint32_t intervalMs;                            //  — интервал мигания в миллисекундах (0 = нет мигания)  
  bool state;                                     //  — текущее состояние (вкл/выкл)  
  TickType_t lastToggle;                          //  — время (в тиках FreeRTOS) последнего переключения  
} chargeBlink{CHARGE_LED_PIN, 0, false, 0},       // Инициализируем экземпляр для индикатора зарядки  
  powerBlink {POWER_LED_PIN,  0, false, 0},       // Инициализируем экземпляр для индикатора питания  
  statusBlink{STATUS_LED_PIN, 0, false, 0};       // Инициализируем экземпляр для индикатора статуса  

// ===== FSM для IMU (удар → свободное падение) =====  
#define IMPACT_THRESHOLD_G     2.5f              // Устанавливаем порог «удара» в g  
#define FREEFALL_THRESHOLD_G   0.5f              // Устанавливаем порог «свободного падения» в g  
#define FREEFALL_TIME_MS       100               // Минимальная длительность свободного падения в миллисекундах  

BMI160GenClass bmi;                               // Создаём объект для управления датчиком BMI323  

// ===== Прототипы задач =====  
void Task_PowerButton(void*);                     // Прототип задачи обработки кнопки питания  
void Task_Battery(void*);                         // Прототип задачи измерения батареи  
void Task_LEDs(void*);                            // Прототип задачи управления светодиодами  
void Task_BLE(void*);                             // Прототип задачи поддержки BLE  
void Task_Controls(void*);                        // Прототип задачи обработки управляющих кнопок  
void Task_IMU(void*);                             // Прототип задачи обнаружения «удар + падение»  

void setup() {  
  Serial.begin(115200);                           // Стартуем Serial-порт на скорости 115200 бод  

  // Настраиваем кнопки как входы с подтяжкой вверх  
  pinMode(POWER_BTN_PIN, INPUT_PULLUP);           //  — кнопка питания  
  pinMode(LEFT_BTN_PIN,  INPUT_PULLUP);           //  — кнопка «налево»  
  pinMode(RIGHT_BTN_PIN, INPUT_PULLUP);           //  — кнопка «направо»  
  pinMode(SIT_BTN_PIN,   INPUT_PULLUP);           //  — кнопка «сесть»  
  pinMode(STAND_BTN_PIN, INPUT_PULLUP);           //  — кнопка «встать/идти»  
  pinMode(READY_BTN_PIN, INPUT_PULLUP);           //  — кнопка «стоп/сброс»  

  // Настраиваем пины светодиодов как выходы  
  pinMode(CHARGE_LED_PIN, OUTPUT);                //  — индикатор зарядки  
  pinMode(POWER_LED_PIN,  OUTPUT);                //  — индикатор питания  
  pinMode(STATUS_LED_PIN, OUTPUT);                //  — индикатор статуса  

  // Инициализируем шину SPI под HSPI (для BMI323)  
  SPI.begin(HSPI_CLK, HSPI_SDO, HSPI_SDI, HSPI_CS);  

  // Конфигурируем АЦП для считывания напряжения батареи  
  analogReadResolution(12);                       //  — 12-битное разрешение  
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db); //  — аттенюация, чтобы измерять до ~3.3 В  

  // Пытаемся инициализировать BMI323 по SPI  
  if (!bmi.begin(HSPI_CS, INT1_PIN)) {        // SPI-CS и пин прерывания
    Serial.println("Ошибка инициализации BMI323");
    currentState = ERROR;
  } else {
    bmi.setAccelerometerRange(BMI160_ACCEL_RANGE_2G); // диапазон ±2g
    bmi.setAccelerometerRate(BMI160_ACCEL_RATE_100HZ);// частота 100 Гц
  }
  // Создаём задачи FreeRTOS, закрепляя их на ядрах ESP32  
  xTaskCreatePinnedToCore(Task_PowerButton, "PowerBtn", 2048, NULL, 2, NULL, 1);  
  xTaskCreatePinnedToCore(Task_Battery,     "Battery",  2048, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(Task_LEDs,        "LEDs",     2048, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(Task_BLE,         "BLE",      4096, NULL, 2, NULL, 0);  
  xTaskCreatePinnedToCore(Task_Controls,    "Controls", 2048, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(Task_IMU,         "IMU",      4096, NULL, 2, NULL, 1);  
}  

void loop() {  
  vTaskDelete(NULL);                             // Удаляем основной (setup/loop) поток, дальше работают только задачи  
}  

// ===== Задача: кнопка питания =====  
void Task_PowerButton(void* pv) {  
  const TickType_t debounce = pdMS_TO_TICKS(50); // Устанавливаем время «дребезга» в 50 мс  
  bool lastState = HIGH;                         // Переменная для хранения предыдущего состояния кнопки  
  TickType_t lastTime = xTaskGetTickCount();     // Время последнего изменения  

  for (;;) {                                     // Бесконечный цикл задачи  
    bool cur = digitalRead(POWER_BTN_PIN);       // Считываем текущее состояние кнопки  
    if (cur != lastState)                        // Если состояние изменилось  
      lastTime = xTaskGetTickCount();            //  — обновляем метку времени  

    // Если после debounce кнопка всё ещё нажата  
    if (xTaskGetTickCount() - lastTime > debounce && cur == LOW && lastState == HIGH) {  
      if (currentState == OFF && batteryPct > 5.0f) {  // Если устройство выключено и заряд >5%  
        currentState = ON;                       //  — включаем устройство  
        powerBlink.intervalMs = 0;               //  — светодиод питания горит постоянно  
        digitalWrite(POWER_LED_PIN, HIGH);       //  — физически включаем LED  
      } else {  
        currentState = OFF;                      // Иначе — выключаем устройство  
        esp_deep_sleep_start();                  //  — переходим в глубокий сон  
      }  
    }  
    lastState = cur;                             // Сохраняем текущее состояние  
    vTaskDelay(pdMS_TO_TICKS(10));               // Ждём 10 мс, чтобы не перегружать ЦП  
  }  
}  

// ===== Задача: измерение батареи =====  
void Task_Battery(void* pv) {  
  const TickType_t interval = pdMS_TO_TICKS(5000);// Интервал опроса 5 сек  

  for (;;) {  
    int raw = analogRead(BATTERY_PIN);            // Читаем значение АЦП  
    float v = raw * (3.3f/4095.0f) * 2.0f;         // Переводим в напряжение (с учётом делителя)  
    float pct = (v*1000 - 3000) * 100.0f / 1200.0f;// Переводим диапазон 3.0–4.2 В → 0–100%  
    batteryPct = constrain(pct, 0.0f, 100.0f);    // Ограничиваем результат 0–100%  

    if (bleKeyboard.isConnected())                // Если BLE-клиент подключён  
      bleKeyboard.setBatteryLevel((int)batteryPct);//  — передаём уровень заряда  

    Serial.printf("Battery: %.1f%%\n", batteryPct);// Выводим значение в консоль  
    vTaskDelay(interval);                         // Ждём следующий цикл опроса  
  }  
}  

// ===== Задача: управление светодиодами =====  
void Task_LEDs(void* pv) {  
  for (;;) {  
    TickType_t now = xTaskGetTickCount();         // Текущее время в тиках  

    // Настраиваем мигание индикатора зарядки  
    if (currentState == CHARGING)                 // Если идёт заряд  
      chargeBlink.intervalMs = (batteryPct >= 100.0f ? 0 : 500);  //  — 500 мс, пока не 100%  
    else {                                        // Иначе  
      chargeBlink.intervalMs = 0;                 //  — гасим индикатор  
      digitalWrite(CHARGE_LED_PIN, LOW);  
    }  

    // Настраиваем мигание индикатора питания  
    if (currentState == ON) {  
      if      (batteryPct < 20.0f) powerBlink.intervalMs = 100;   //  — критически низкий заряд  
      else if (batteryPct < 50.0f) powerBlink.intervalMs = 300;   //  — низкий заряд  
      else                          powerBlink.intervalMs = 0,   //  — нормальный заряд  
                                    digitalWrite(POWER_LED_PIN, HIGH);  
    }  

    // Настраиваем индикатор статуса BLE/ошибки  
    if (currentState == ERROR)                   // Если в состоянии ошибки  
      statusBlink.intervalMs = 100;              //  — быстро мигаем  
    else if (bleKeyboard.isConnected()) {        // Если BLE подключено  
      statusBlink.intervalMs = 0;                //  — постоянно горит  
      digitalWrite(STATUS_LED_PIN, HIGH);  
    } else                                       // Иначе (нет BLE)  
      statusBlink.intervalMs = 300;              //  — медленно мигаем  

    // Лямбда-функция для обновления одного блинкера  
    auto updateBlink = [&](Blinker &b){  
      if (b.intervalMs == 0) return;             // Если мигание не требуется — выходим  
      if (now - b.lastToggle >= pdMS_TO_TICKS(b.intervalMs)) { // Если пора переключить  
        b.lastToggle = now;                      //  — обновляем время  
        b.state = !b.state;                      //  — меняем состояние  
        digitalWrite(b.pin, b.state);            //  — физически переключаем LED  
      }  
    };  

    updateBlink(chargeBlink);                    // Обновляем индикатор зарядки  
    updateBlink(powerBlink);                     // Обновляем индикатор питания  
    updateBlink(statusBlink);                    // Обновляем индикатор статуса  

    vTaskDelay(pdMS_TO_TICKS(50));               // Ждём 50 мс перед следующим обновлением  
  }  
}  

// ===== Задача: поддержка BLE =====  
void Task_BLE(void* pv) {  
  bleKeyboard.begin();                            // Инициализируем BLE-клавиатуру  
  for (;;) {  
    if (!bleKeyboard.isConnected()) {             // Если клиент отключился  
      bleKeyboard.end();                          //  — завершаем сервис  
      bleKeyboard.begin();                        //  — перезапускаем сервис  
    }  
    vTaskDelay(pdMS_TO_TICKS(10000));             // Пауза 10 сек перед проверкой снова  
  }  
}  

// ===== Задача: обработка управляющих кнопок =====  
void Task_Controls(void* pv) {  
  struct Btn { uint8_t pin; uint16_t key; };      // Структура «пин → код клавиши»  
  const Btn actions[] = {                         // Массив действий  
    {LEFT_BTN_PIN,  KEY_LEFT_ARROW},              //  — влево  
    {RIGHT_BTN_PIN, KEY_RIGHT_ARROW},             //  — вправо  
    {SIT_BTN_PIN,   's'},                         //  — сесть  
    {STAND_BTN_PIN, 'w'},                         //  — встать/идти  
    {READY_BTN_PIN, 'r'}                          //  — стоп/сброс  
  };  
  const TickType_t debounce = pdMS_TO_TICKS(50);  // Дебаунс для кнопок 50 мс  
  TickType_t lastTime[5] = {};                    // Массив времён последнего изменения  
  bool lastState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH}; // Массив предыдущих состояний  

  for (;;) {  
    if (currentState == ON && bleKeyboard.isConnected()) { // Только если включено и BLE подключено  
      for (int i = 0; i < 5; i++) {  
        bool cur = digitalRead(actions[i].pin);  // Считываем состояние кнопки  
        if (cur != lastState[i])                 // Если изменилось  
          lastTime[i] = xTaskGetTickCount();     //  — сбрасываем таймер  
        if (xTaskGetTickCount() - lastTime[i] > debounce && cur == LOW && lastState[i] == HIGH) {  
          bleKeyboard.press(actions[i].key);     //  — нажимаем виртуальную клавишу  
          bleKeyboard.release(actions[i].key);   //  — отпускаем её  
        }  
        lastState[i] = cur;                      // Обновляем предыдущее состояние  
      }  
    }  
    vTaskDelay(pdMS_TO_TICKS(10));               // Ждём 10 мс между итерациями  
  }  
}  

// ===== Задача: обнаружение «удар + свободное падение» =====  
void Task_IMU(void* pv) {  
  const TickType_t ffTicks = pdMS_TO_TICKS(FREEFALL_TIME_MS); // Время свободного падения в тиках  
  bool impactDetected = false;                        // Флаг, что удар уже произошёл  
  TickType_t ffStart = 0;                             // Время начала падения  

  for (;;) {  
                                  // Переменные для акселерометра  
    int16_t rawX, rawY, rawZ;
    bmi.getAcceleration(&rawX, &rawY, &rawZ); // получаем 16-битные сырые данные
    
    // переводим сырые данные в ускорение в g
    float ax = rawX * (2.0f / 32768.0f);
    float ay = rawY * (2.0f / 32768.0f);
    float az = rawZ * (2.0f / 32768.0f);
             // Читаем ускорение по трем осям  
    float mag = sqrt(ax*ax + ay*ay + az*az);          // Вычисляем общую величину ускорения (модуль)  

    if (!impactDetected) {                            // Если удар ещё не зафиксирован  
      if (mag >= IMPACT_THRESHOLD_G) {                //  — проверяем порог удара  
        impactDetected = true;                        //    фиксируем удар  
        ffStart = 0;                                  //    сбрасываем таймер падения  
        Serial.println("Удар обнаружен, ожидаем падение");  
      }  
    } else {                                          // Иначе (удар уже был)  
      if (mag <= FREEFALL_THRESHOLD_G) {              //  — если ускорение упало ниже порога падения  
        if (ffStart == 0)                             //    и таймер падения ещё не запущен  
          ffStart = xTaskGetTickCount();              //      — отмечаем время начала падения  
        else if (xTaskGetTickCount() - ffStart >= ffTicks) { //    или если прошло достаточно времени  
          Serial.println("Свободное падение подтверждено, отправляю STOP");  
          if (bleKeyboard.isConnected())  
                        //      — если BLE подключено  
            bleKeyboard.press(KEY_F11);                     //        отправляем команду стоп (‘r’)  
          impactDetected = false;                     //      — сбрасываем детектор для нового цикла  
        }  
      } else {                                        //  — если ускорение снова превысило FREEFALL_THRESHOLD_G  
        ffStart = 0;                                  //    сбрасываем таймер падения  
      }  
    }  

    vTaskDelay(pdMS_TO_TICKS(20));                    // Пауза ~20 мс (~50 Гц) перед следующим чтением  
  }  
}  
