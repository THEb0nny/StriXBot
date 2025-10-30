// https://alexgyver.ru/lessons/pid/
// https://github.com/GyverLibs/uPID
// https://github.com/GyverLibs/GyverMotor
// https://github.com/GyverLibs/GTimer
// https://github.com/GyverLibs/EncButton
// https://github.com/GyverLibs/StringUtils
// https://wiki.amperka.ru/products:openmv-cam-h7-plus

#include <SoftwareSerial.h>
#include <GyverMotor2.h>
#include <uPID.h>
#include <GTimer.h>
#include <EncButton.h>
#include <StringUtils.h>

using namespace su;

#define PRINT_DT_ERR_U_DEBUG 1 // Печать значений dt, error и u
#define MOTORS_WORK 1 // Работа моторов

#define CAM_RX_PIN 12 // RX (подключить к TX P4 OpenMV)
#define CAM_TX_PIN 11 // TX (подключить к RX P5 OpenMV)

#define BT_RX_PIN 6 // RX блютуза
#define BT_TX_PIN 10 // TX блютуза

#define MOT_LEFT_IN1_PIN 7 // Пин управляющий направлением вращения левого мотора
#define MOT_LEFT_IN2_PIN 8 // Пин управляющий направлением вращения левого мотора
#define MOT_LEFT_PWM_PIN 9 // Пин ШИМ левого мотора

#define MOT_RIGHT_IN1_PIN 4 // Пин управляющий направлением вращения правого мотора
#define MOT_RIGHT_IN2_PIN 5 // Пин управляющий направлением вращения правого мотора
#define MOT_RIGHT_PWM_PIN 3 // Пин ШИМ правого мотора

#define RESET_BTN_PIN 2 // Пин кнопки для старта, мягкого перезапуска

#define KP 1.0 // Стартовые значения KP pid регулятора
#define KI 0 // Стартовые значения KI pid регулятора
#define KD 0 // Стартовые значения KD pid регулятора

SoftwareSerial OpenMVSerial(CAM_RX_PIN, CAM_TX_PIN);
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

uPID pid(D_INPUT | I_SATURATE); // Инициализируем регулятор и устанавливаем коэффициенты регулятора
GTimer<millis> regTmr;  // Инициализация объекта таймера цикла регулирования в нужном количестве мс
Button btn(RESET_BTN_PIN, INPUT, LOW); // Инициализация объекта простой кнопки

GMotor2<DRIVER3WIRE> motorLeft(MOT_LEFT_IN1_PIN, MOT_LEFT_IN2_PIN, MOT_LEFT_PWM_PIN); // Объект левого мотора
GMotor2<DRIVER3WIRE> motorRight(MOT_RIGHT_IN1_PIN, MOT_RIGHT_IN2_PIN, MOT_RIGHT_PWM_PIN); // Объект правого мотора

unsigned long currTime, prevTime; // Время
int dt = 10; // Время

int speed = 100; // Инициализируем переменную скорости
float error = 0; // Ошибка регулирования

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void btnISR() {
  btn.pressISR(); // Функция сообщает библиотеке, что кнопка была нажата вне tick()
  Serial.println("SOFT RESET");
  Serial.flush(); // Дождаться отправки всех данных по UART
  softResetFunc();
}

void setup() {
  Serial.begin(57600); // Инициализация скорости общения по монитору порта
  Serial.setTimeout(10); // Задаёт время ожидания данных, поступающих через последовательный интерфейс
  OpenMVSerial.begin(57600); // Инициализация скорости общения с камерой
  OpenMVSerial.setTimeout(5);
  BTSerial.begin(57600); // Инициализация скорости общения с блютузом
  BTSerial.setTimeout(5);
  Serial.println();
  if (Serial) Serial.println("STARTS...");
  motorLeft.reverse(1); // Направление вращение левого мотора
  motorRight.reverse(1); // Направление вращения правого мотора
  motorLeft.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на левый мотор
  motorRight.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на правый мотор
  motorLeft.setDeadtime(5); // Установить deadtime в микросекундах
  motorRight.setDeadtime(5); // Установить deadtime в микросекундах
  pid.setKp(KP); // Установка стартовых значений пид регулятора
  pid.setKi(KI);
  pid.setKd(KD);
  pid.outMin = -255; // Нижний предел регулирования
  pid.outMax = 255; // Верхний предел регулирования
  pid.setDt(dt);
  regTmr.setMode(GTMode::Interval); // Настроем режим таймера регулирования на интервал
  regTmr.setTime(dt);
  while (millis() < 250); // Время после старта для возможности запуска, защита от перезагрузки и старта кода сразу
  if (Serial) Serial.println("Ready... press btn to run");
  PauseUntilBtnPressed(); // Ждём нажатия для старта
  EIFR = (1 << digitalPinToInterrupt(RESET_BTN_PIN)); // Очищаем флаг прерывания, иначе сработает прерывание и будет перезагрузка
  delay(50); // Антидребезг перед активацией прерывания
  attachInterrupt(digitalPinToInterrupt(RESET_BTN_PIN), btnISR, FALLING); // Назначит прерывание на кнопку
  if (Serial) Serial.println("Run!");
  regTmr.start(); // Запускаем таймер цикла регулирования
  // Записываем время перед стартом loop
  prevTime = millis();
}

void loop() {
  ParseFromSerialInputValues(OpenMVSerial, true); // Парсинг значений из Serial
  ParseFromSerialInputValues(BTSerial, true); // Парсинг значений из Serial

  if (regTmr.tick()) { // Раз в N мсек выполнять регулирование
    currTime = millis();
    dt = currTime - prevTime;
    prevTime = currTime;

    pid.setpoint = error; // Передаём ошибку регулирования
    pid.setDt(dt); // Установка dt для регулятора
    float u = pid.compute(0); // Управляющее воздействие с регулятора

    if (MOTORS_WORK) MotorsControl(u, speed); // Для управления моторами регулятором
    
    // Для отладки основной информации о регулировании
    if (Serial && PRINT_DT_ERR_U_DEBUG) {
      Serial.println("dt: " + String(dt) + "\terror: " + String(error) + "\tu: " + String(u));
    }
  }
}

// Управление двумя моторами
void MotorsControl(float dir, int speed) {
  int lMotorSpeed = speed + dir, rMotorSpeed = speed - dir;
  // float z = (float) speed / max(abs(lMotorSpeed), abs(rMotorSpeed)); // Вычисляем отношение желаемой мощности к наибольшей фактической
  // lMotorSpeed *= z, rMotorSpeed *= z;
  motorLeft.setSpeed(lMotorSpeed);
  motorRight.setSpeed(rMotorSpeed);
}

// Ждать нажатия
void PauseUntilBtnPressed() {
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.release()) break; // Произошло нажатие
  }
}

// Парсинг входящих по Serial значений
void ParseFromSerialInputValues(Stream& serial, bool debug) {
  static char input[64];

  if (!serial.available()) return; // Проверяем, есть ли вообще данные

  int amount = serial.readBytesUntil(';', input, sizeof(input) - 1); // Читаем строку до ';'
  if (amount <= 0) return;
  input[amount] = '\0';

  // Убираем \r и \n прямо в исходной строке
  for (char* p = input; *p; p++) {
    if (*p == '\r' || *p == '\n') {
      *p = '\0';
      break;
    }
  }

  // Оборачиваем в Text и сразу очищаем от пробелов, \r и \n
  Text t(input);
  t.trim();

  // Пропускаем пустые строки
  if (!t || t.length() == 0) return;

  if (debug) {
    Serial.print(F("RAW: "));
    Serial.println(t.str());
  }

  Text parts[12]; // Разделяем по запятым на отдельные пары "ключ:значение"
  uint16_t partsCnt = t.split(parts, 12, ',');

  for (uint16_t i = 0; i < partsCnt; ++i) {
    Text pair = parts[i].trim();
    if (!pair) continue;

    Text tokens[2];
    uint16_t tokenCount = pair.split(tokens, 2, ':'); // ':' — разделитель ключ/значение
    if (tokenCount < 2) continue;

    char keyBuf[24], valBuf[24]; // Локальные буферы — безопасно, размер подбирай по нуждам
    // Скопировать токены в буферы и гарантировать '\0'
    tokens[0].trim().toStr(keyBuf, sizeof(keyBuf), true); // true — гарантировать терминацию
    tokens[1].trim().toStr(valBuf, sizeof(valBuf), true);

    if (Serial && debug) {
      Serial.println(String(F("key: [")) + keyBuf + F("], value: [") + valBuf + F("]"));
    }

    // Обработка ключей — теперь надёжно
    if (strcmp(keyBuf, "kp") == 0) pid.setKp(strToFloat(valBuf));
    else if (strcmp(keyBuf, "ki") == 0) pid.setKi(strToFloat(valBuf));
    else if (strcmp(keyBuf, "kd") == 0) pid.setKd(strToFloat(valBuf));
    else if (strcmp(keyBuf, "speed") == 0) speed = strToInt<int>(valBuf);
    else if (strcmp(keyBuf, "error") == 0) error = strToFloat(valBuf);
  }

  if (Serial && debug) Serial.println(F("PARSING DONE"));
}