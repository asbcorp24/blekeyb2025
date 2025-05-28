
 

#include <BleKeyboard.h>                // Подключаем библиотеку для эмуляции BLE-клавиатуры  
#include <BMI160Gen.h>                  // Подключаем библиотеку для работы с акселерометром BMI160/323  
#include <SPI.h>                        // Подключаем библиотеку для работы с интерфейсом SPI  
//#include <NimBLEDevice.h>              // Подключаем базовую библиотеку NimBLE для BLE-стека  
#include "freertos/FreeRTOS.h"          // Подключаем заголовок ядра FreeRTOS  
#include "freertos/task.h"             // Подключаем API для создания и управления задачами FreeRTOS  
#include <Keypad.h>
// ===== Пины подключения =====  
#define HSPI_CLK        14             // Номер пина CLK для шины HSPI (датчик BMI323)  
#define HSPI_SDI        12             // Номер пина MOSI (SDI) для шины HSPI (датчик BMI323)  
#define HSPI_SDO        13             // Номер пина MISO (SDO) для шины HSPI (датчик BMI323)  
#define HSPI_CS         25             // Номер пина CS (Chip Select) для шины HSPI (датчик BMI323)  
#define INT1_PIN        26             // Номер пина для входа прерывания INT1 от датчика BMI323  
#define INT2_PIN        27             // Номер пина для входа прерывания INT2 от датчика BMI323  

//#define CHARGE_LED_PIN  22             // Номер пина для светодиода индикатора зарядки  
#define POWER_LED_PIN   23             // Номер пина для светодиода индикатора питания  
#define STATUS_LED_PIN  22             // Номер пина для светодиода индикатора статуса (BLE/ошибка)  

#define BATTERY_PIN     34             // Номер пина ADC для измерения напряжения батареи  


#define LEFT_BTN_PIN    5             // Номер пина для кнопки «налево»  
#define RIGHT_BTN_PIN   18             // Номер пина для кнопки «направо»  
#define SIT_BTN_PIN     19             // Номер пина для кнопки «сесть»  
#define STAND_BTN_PIN   17              // Номер пина для кнопки «встать/идти»  
#define READY_BTN_PIN   16              // Номер пина для кнопки «стоп/сброс»  
#define STD_PIN         4   // дорожка 6

  const byte ROWS = 3; //four rows
const byte COLS = 3; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'A','B','C'},
  {'D','E','F'},
  {'G','I','Y'},
  
};
byte rowPins[ROWS] = {LEFT_BTN_PIN,RIGHT_BTN_PIN,SIT_BTN_PIN}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {STAND_BTN_PIN,READY_BTN_PIN,STD_PIN}; //connect to the column pinouts of the keypad
//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
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
} //chargeBlink{CHARGE_LED_PIN, 0, false, 0},       // Инициализируем экземпляр для индикатора зарядки  
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
 // pinMode(POWER_BTN_PIN, INPUT_PULLUP);           //  — кнопка питания  
 /* pinMode(LEFT_BTN_PIN,  INPUT);           //  — кнопка «налево»  
  pinMode(RIGHT_BTN_PIN, INPUT);           //  — кнопка «направо»  
  pinMode(SIT_BTN_PIN,   INPUT);           //  — кнопка «сесть»  
  pinMode(STAND_BTN_PIN, INPUT);           //  — кнопка «встать/идти»  
  pinMode(READY_BTN_PIN, INPUT);           //  — кнопка «стоп/сброс»  */

  // Настраиваем пины светодиодов как выходы  
  //pinMode(CHARGE_LED_PIN, OUTPUT);                //  — индикатор зарядки  
   pinMode(POWER_LED_PIN,  OUTPUT);                //  — индикатор питания  
   // digitalWrite(POWER_LED_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);                //  — индикатор статуса  

  // Инициализируем шину SPI под HSPI (для BMI323)  
  SPI.begin(HSPI_CLK, HSPI_SDO, HSPI_SDI, HSPI_CS);  

  // Конфигурируем АЦП для считывания напряжения батареи  
  analogReadResolution(12);                       //  — 12-битное разрешение  
  analogSetPinAttenuation(BATTERY_PIN, ADC_ATTENDB_MAX); //  — аттенюация, чтобы измерять до ~3.3 В  

   // Пытаемся инициализировать BMI323 по SPI  
/*   if (!bmi.begin(HSPI_CS, INT1_PIN)) {        // SPI-CS и пин прерывания
    Serial.println("Ошибка инициализации BMI323");
    currentState = ERROR;
  } else {
    bmi.setAccelerometerRange(BMI160_ACCEL_RANGE_2G); // диапазон ±2g
    bmi.setAccelerometerRate(BMI160_ACCEL_RATE_100HZ);// частота 100 Гц
  }  */
  // Создаём задачи FreeRTOS, закрепляя их на ядрах ESP32  
 // xTaskCreatePinnedToCore(Task_PowerButton, "PowerBtn", 2048, NULL, 2, NULL, 1);  
   currentState = ON;                       //  — включаем устройство  
        powerBlink.intervalMs = 0;               //  — светодиод питания горит постоянно  
      digitalWrite(POWER_LED_PIN,HIGH); //вернуть
  xTaskCreatePinnedToCore(Task_Battery,     "Battery",  4096, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(Task_LEDs,        "LEDs",     2048, NULL, 1, NULL, 1);  
  xTaskCreatePinnedToCore(Task_BLE,         "BLE",      4096, NULL, 2, NULL, 0);  
  xTaskCreatePinnedToCore(Task_Controls,    "Controls", 4096, NULL, 1, NULL, 1);  
 // xTaskCreatePinnedToCore(Task_IMU,         "IMU",      4096, NULL, 2, NULL, 1);  
}  

void loop() {  
  vTaskDelete(NULL);                             // Удаляем основной (setup/loop) поток, дальше работают только задачи  
}  

// ===== Задача: кнопка питания =====  
/* void Task_PowerButton(void* pv) {  
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
}   */

// ===== Задача: измерение батареи =====  
void Task_Battery(void* pv) {  
  
  const TickType_t interval = pdMS_TO_TICKS(25000);// Интервал опроса 5 сек  

  for (;;) {  
    int raw = analogRead(BATTERY_PIN);        
       // Читаем значение АЦП  
    float v = raw * (3.3f/4095.0f);         // Переводим в напряжение (с учётом делителя)  
      Serial.println(v);// Выводим значение в консоль 
   
    float pct = v/4.2*100;// Переводим диапазон 3.0–4.2 В → 0–100%  
    batteryPct = constrain(pct, 0.0f, 100.0f);    // Ограничиваем результат 0–100%  

    if (bleKeyboard.isConnected())
    if(batteryPct>80){ bleKeyboard.write(KEY_F6);}
    else if(batteryPct>60){ bleKeyboard.write(KEY_F7);}
    else if (batteryPct>40){ bleKeyboard.write(KEY_F8);}
    else if(batteryPct>20){ bleKeyboard.write(KEY_F9);}              // Если BLE-клиент подключён  
      bleKeyboard.setBatteryLevel((int)batteryPct);//  — передаём уровень заряда  

    Serial.printf("Battery: %.1f%%\n", batteryPct);// Выводим значение в консоль  
    vTaskDelay(interval);                         // Ждём следующий цикл опроса  
  }  
}  

// ===== Задача: управление светодиодами =====  
void Task_LEDs(void* pv) {  
  for (;;) {  
    TickType_t now = xTaskGetTickCount();         // Текущее время в тиках  

   /*  // Настраиваем мигание индикатора зарядки  
    if (currentState == CHARGING)                 // Если идёт заряд  
      chargeBlink.intervalMs = (batteryPct >= 100.0f ? 0 : 500);  //  — 500 мс, пока не 100%  
    else {                                        // Иначе  
      chargeBlink.intervalMs = 0;                 //  — гасим индикатор  
      digitalWrite(CHARGE_LED_PIN, LOW);  
    }   */

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

   // updateBlink(chargeBlink);                    // Обновляем индикатор зарядки  
    updateBlink(powerBlink);                     // Обновляем индикатор питания  
    updateBlink(statusBlink);                    // Обновляем индикатор статуса  

    vTaskDelay(pdMS_TO_TICKS(50));               // Ждём 50 мс перед следующим обновлением  
  }  
}  

// ===== Задача: поддержка BLE =====  
void Task_BLE(void* pv) {  
  bleKeyboard.begin();   
   Serial.println("стартуем BLE");                               // Инициализируем BLE-клавиатуру  
  for (;;) {  
     Serial.println("пинг");      
   /* if (!bleKeyboard.isConnected()) {    
      Serial.println("стартуем BLE");           // Если клиент отключился  
      bleKeyboard.end();                          //  — завершаем сервис  
      bleKeyboard.begin();                        //  — перезапускаем сервис  
    }*/
     if (!bleKeyboard.isConnected()) {
      Serial.println("ожидаем клиента...");
    } else {
      Serial.println("клиент подключен");
    }  
    vTaskDelay(pdMS_TO_TICKS(10000));             // Пауза 10 сек перед проверкой снова  
  }  
}  


void Task_Controls(void* pv) {
 for (;;) {
    // чтение из Keypad
    char customKey = customKeypad.getKey();

    // если нажато что-то, отличное от NO_KEY
    if (customKey != NO_KEY) {
      // выведем в консоль букву-код
      Serial.printf("Keypad: %c\n", customKey);

      // в зависимости от символа шлём нужную F-клавишу
      switch (customKey) {
        //case 'Y': bleKeyboard.press(KEY_F1);Serial.println("F1"); break;  // 1–4
        case 'Y': bleKeyboard.write(KEY_F1);Serial.println("F1"); break;  // 1–4
        case 'I': bleKeyboard.write(KEY_F2);Serial.println("F2"); break;  // 2–5
        case 'E': bleKeyboard.write(KEY_F3);Serial.println("F3"); break;  // 2–6
        case 'B': bleKeyboard.write(KEY_F4);Serial.println("F4"); break;  // 2–4
        case 'G': bleKeyboard.write(KEY_F5);Serial.println("F5"); break;  // 3–4
        default:
          // если попало что-то лишнее — игнорируем
          break;
      }
    }

    // даём другим задачам пожить, 10 мс
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* void Task_Controls2(void* pv) {
  // Номера GPIO, к которым подключены дорожки клавиатуры
 

  // Каждая кнопка — это пара (i, j) дорожек, плюс код клавиши
  struct KeyMap { uint8_t a, b; uint16_t key; };
  const KeyMap keys[5] = {
    { 1, 4, KEY_F1 },  // дорожки 1–4 → F1
    { 2, 5, KEY_F2 },  // дорожки 2–5 → F2
    { 2, 6, KEY_F3 },  // дорожки 2–6 → F3
    { 2, 4, KEY_F4 },  // дорожки 2–4 → F4
    { 3, 4, KEY_F5 }   // дорожки 3–4 → F5
  };

  const TickType_t debounce = pdMS_TO_TICKS(50);
  TickType_t lastTime[5] = {0};
  bool lastState[5]  = {false, false, false, false, false};

  // Инициализация: все дорожки — входы с подтяжкой
  for (uint8_t t = 0; t < 6; ++t) {
    pinMode(trackPins[t], INPUT_PULLUP);
  }

  for (;;) {
    if (currentState == ON && bleKeyboard.isConnected()) {
      // Для каждой дорожки i делаем output LOW, остальные — input pullup
      for (uint8_t i = 0; i < 6; ++i) {
        // 1) Опустить линию i
        pinMode(trackPins[i], OUTPUT);
        digitalWrite(trackPins[i], LOW);
        // 2) Все остальные — входы с подтяжкой
        for (uint8_t j = 0; j < 6; ++j) {
          if (j != i) {
            pinMode(trackPins[j], INPUT_PULLUP);
          }
        }
        // 3) Смотрим, какие из остальных подтянулись вниз
        for (uint8_t k = 0; k < 5; ++k) {
          uint8_t a = keys[k].a - 1;
          uint8_t b = keys[k].b - 1;
          bool pressed = false;
          // кнопка k замыкает дорожки a и b
          if ((a == i && digitalRead(trackPins[b]) == LOW) ||
              (b == i && digitalRead(trackPins[a]) == LOW)) {
            pressed = true;
          }
          // Дребезг
          if (pressed != lastState[k]) {
            lastTime[k] = xTaskGetTickCount();
          }
          if (xTaskGetTickCount() - lastTime[k] > debounce) {
            if (pressed && !lastState[k]) {
              // подтверждённое нажатие
              Serial.println( keys[k].key) ;
              bleKeyboard.press(keys[k].key);
              bleKeyboard.release(keys[k].key);
            }
            lastState[k] = pressed;
          }
        }
        // 4) Сброс: делаем i обратно входом
        pinMode(trackPins[i], INPUT_PULLUP);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // пауза 10 мс
  }
}
 */

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
            bleKeyboard.write(KEY_F11);                     //        отправляем команду стоп (‘r’)  
          impactDetected = false;                     //      — сбрасываем детектор для нового цикла  
        }  
      } else {                                        //  — если ускорение снова превысило FREEFALL_THRESHOLD_G  
        ffStart = 0;                                  //    сбрасываем таймер падения  
      }  
    }  

    vTaskDelay(pdMS_TO_TICKS(20));                    // Пауза ~20 мс (~50 Гц) перед следующим чтением  
  }  
}