#include <BleKeyboard.h>
#include <WiFi.h> // Для получения MAC-адреса
#include <NimBLEDevice.h>
// Переменные для BLE-клавиатуры
BleKeyboard bleKeyboard;

// Определяем пины для кнопок
const int buttonPins[] = {2, 4, 5, 18, 19, 21}; // Укажите свои пины
const int numButtons = 6;

// Переменные для хранения состояния кнопок
bool buttonStates[numButtons] = {false, false, false, false, false, false};
unsigned long lastButtonPressTime[numButtons] = {0, 0, 0, 0, 0, 0};
const unsigned long buttonDebounceDelay = 50; // Задержка для устранения дребезга

// Привязываем коды клавиш к кнопкам
uint8_t keyCodes[] = {
    KEY_F1, // Кнопка 1 отправляет F1
    KEY_F2, // Кнопка 2 отправляет F2
    'A',    // Кнопка 3 отправляет A
    KEY_F4, // Кнопка 4 отправляет F4
    KEY_F5, // Кнопка 5 отправляет F5
    KEY_F6  // Кнопка 6 отправляет F6
};

// Таймер для отслеживания подключения BLE
unsigned long lastConnectAttempt = 0;
const unsigned long connectTimeout = 10000; // Таймаут 10 секунд
bool bleRestarted = false;

// Функция для формирования имени устройства
String generateDeviceName() {
  uint8_t mac[6];
  WiFi.macAddress(mac); // Получаем MAC-адрес
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String deviceName = "exoc-" + String(macStr);
  return deviceName;
}
  String deviceName = generateDeviceName();
void setup() {
  Serial.begin(115200);
  Serial.println("Запуск...");

  // Генерация имени устройства

  Serial.println("Имя устройства: " + deviceName);

  // Инициализация BLE-клавиатуры с динамическим именем
  
  bleKeyboard = BleKeyboard(deviceName.c_str(), "exoc-manufacturer", 100); // Пример имени производителя и уровня заряда
  bleKeyboard.begin();

  // Настройка кнопок
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP); // Используем встроенный подтягивающий резистор
  }

  lastConnectAttempt = millis(); // Устанавливаем начальное время
}

void loop() {
  if (bleKeyboard.isConnected()) {
    if (bleRestarted) {
      Serial.println("Клиент подключен. Состояние BLE восстановлено.");
      bleRestarted = false; // Сбрасываем флаг
    }

    // Обработка нажатий кнопок
    for (int i = 0; i < numButtons; i++) {
      bool currentState = digitalRead(buttonPins[i]) == LOW; // Нажатие кнопки
      if (currentState != buttonStates[i] && millis() - lastButtonPressTime[i] > buttonDebounceDelay) {
        buttonStates[i] = currentState;
        lastButtonPressTime[i] = millis();

        if (currentState) {
          Serial.print("Кнопка ");
          Serial.print(i + 1);
          Serial.println(" нажата!");

          // Отправляем соответствующий код клавиши
          bleKeyboard.press(keyCodes[i]);
          bleKeyboard.release(keyCodes[i]);
        }
      }
    }

    lastConnectAttempt = millis(); // Обновляем таймер подключения
  } else {
    Serial.println("Клиент отключен. Ожидание подключения...");

    // Перезапуск BLE, если таймаут истек
    if (!bleRestarted && millis() - lastConnectAttempt > connectTimeout) {
      Serial.println("Время подключения истекло. Перезапуск BLE...");
      bleKeyboard.end();     // Завершаем работу BLE
      bleKeyboard.begin();   // Инициализируем BLE заново
      NimBLEDevice::init(deviceName.c_str()); // BLE инициируется один раз
      bleRestarted = true;   // Устанавливаем флаг перезапуска
      lastConnectAttempt = millis(); // Сбрасываем таймер
    }
  }

  delay(10); // Основной цикл
}
