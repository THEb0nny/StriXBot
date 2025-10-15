// https://alexgyver.ru/lessons/pid/
// https://github.com/GyverLibs/uPID
// https://github.com/GyverLibs/GyverMotor
// https://github.com/GyverLibs/GTimer
// https://github.com/GyverLibs/EncButton

#include <SoftwareSerial.h>
#include <GyverMotor2.h>
#include <uPID.h>
#include <GTimer.h>
#include <EncButton.h>
#include <GParser.h>

#define PRINT_DT_ERR_U_DEBUG false // Печать значений dt, error, и TRUE

#define MOT_LEFT_IN1_PIN 7 // Пин управляющий направлением вращения левого мотора
#define MOT_LEFT_IN2_PIN 8 // Пин управляющий направлением вращения левого мотора
#define MOT_LEFT_PWM_PIN 9 // Пин ШИМ левого мотора

#define MOT_RIGHT_IN1_PIN 5 // Пин управляющий направлением вращения правого мотора
#define MOT_RIGHT_IN2_PIN 4 // Пин управляющий направлением вращения правого мотора
#define MOT_RIGHT_PWM_PIN 3 // Пин ШИМ правого мотора

#define RESET_BTN_PIN 3 // Пин кнопки для старта, мягкого перезапуска

#define KP 1.0 // Стартовые значения KP pid регулятора
#define KI 0 // Стартовые значения KI pid регулятора
#define KD 0 // Стартовые значения KD pid регулятора

unsigned long currTime, prevTime; // Время
int dt = 1; // Время

int speed = 255; // Инициализируем переменную скорости

uPID pid(D_INPUT | I_SATURATE); // Инициализируем регулятор и устанавливаем коэффициенты регулятора
GTimer<millis> regTmr;  // Инициализация объекта таймера цикла регулирования в нужном количестве мс
Button btn(RESET_BTN_PIN, INPUT, LOW); // Инициализация объекта простой кнопки

GMotor2<DRIVER3WIRE> motorLeft(MOT_LEFT_IN1_PIN, MOT_LEFT_IN2_PIN, MOT_LEFT_PWM_PIN); // Объект левого мотора
GMotor2<DRIVER3WIRE> motorRight(MOT_RIGHT_IN1_PIN, MOT_RIGHT_IN2_PIN, MOT_RIGHT_PWM_PIN); // Объект правого мотора

void(* softResetFunc) (void) = 0; // Функция мягкого перезапуска

void btnISR() {
  btn.pressISR(); // Функция сообщает библиотеке, что кнопка была нажата вне tick()
  Serial.println("Press");
}

void setup() {
  Serial.begin(115200); // Инициализация скорости общения по монитору порта
  Serial.setTimeout(10); // Позволяет задать время ожидания данных, поступающих через последовательный интерфейс
  Serial.println();
  motorLeft.reverse(1); // Направление вращение левого мотора
  motorRight.reverse(0); // Направление вращения правого мотора
  motorLeft.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на левый мотор
  motorRight.setMinDuty(10); // Минимальный сигнал (по модулю), который будет подан на правый мотор
  pid.setKp(KP); pid.setKi(KI); pid.setKd(KD); // Установка стартовых значений пид регулятора
  pid.outMin = -255; // Нижний предел регулирования
  pid.outMax = 255; // Верхний предел регулирования
  pid.setDt(dt);
  regTmr.setMode(GTMode::Interval); // Настроем режим таймера регулирования на интервал
  regTmr.setTime(dt);
  attachInterrupt(0, btnISR, FALLING); // Назначит прерывание на кнопку
  while (millis() < 500); // Время после старта для возможности запуска, защита от перезагрузки и старта кода сразу
  Serial.println("Ready... press btn to start");
  PauseUntilBtnPressed("Start!"); // Ждём нажатия для старта
  regTmr.start(); // Запускаем таймер цикла регулирования
  // Записываем время перед стартом loop
  currTime = millis();
  prevTime = currTime;
}

void loop() {
  // CheckBtnClickToReset(); // Вызываем функцию опроса кнопки
  ParseFromSerialInputValues(true); // Парсинг значений из Serial

  if (regTmr.tick()) { // Раз в N мсек выполнять
    currTime = millis();
    dt = currTime - prevTime;
    prevTime = currTime;

    float error = 0;
    pid.setpoint = error; // Передаём ошибку регулирования
    pid.setDt(dt); // Установка dt для регулятора
    float u = pid.compute(0); // Управляющее воздействие с регулятора

    MotorsControl(u, speed); // Для управления моторами регулятором
    
    // Для отладки основной информации о регулировании
    if (PRINT_DT_ERR_U_DEBUG) {
      // Serial.println("dt: " + String(dt) + "\terror: " + String(error) + "\tu: " + String(u));
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

void PauseUntilBtnPressed(String str) {
  while (true) { // Ждём нажатие кнопки для старта
    btn.tick(); // Опрашиваем кнопку
    if (btn.press()) break; // Произошло нажатие
  }
  Serial.println(str);
}

// Функция опроса о нажатии кнопки
void CheckBtnClickToReset() {
  btn.tick(); // Опрашиваем кнопку в первый раз
  if (btn.press()) { // Произошло нажатие
    Serial.println("Btn press and reset");
    delay(50); // Нужна задержка иначе не выведет сообщение
    softResetFunc(); // Если клавиша нажата, то сделаем мягкую перезагрузку
  }
}

// Парсинг значений из Serial
void ParseFromSerialInputValues(bool debug) {
  if (Serial.available() > 2) { // Если что-то прислали
    char inputStr[64]; // Массив символов для записи из Serial
    int amount = Serial.readBytesUntil(';', inputStr, 64); // Считать посимвольно до символа конца пакета точки с запятой и записать количество полученных байт в переменную
    inputStr[amount] = "\0"; // Если отправляющее устройство не отправит нулевой символ, то он не запишется в буффер и вывод строк будет некорректным, решение дописать вручную и т.о. закрываем строку
    GParser data(inputStr, ','); // Парсим массив символов по символу запятой
    int am = data.split(); // Получаем количество данных, внимание, ломает строку!
    for (int i = 0; i < am; i++) {
      String tmpStr = data[i];
      tmpStr.replace(" ", ""); // Удалить пробел, если он был введёт по ошибке
      tmpStr.trim(); // Удаление ведущими и конечные пробелы
      char tmpCharArr[tmpStr.length()];
      tmpStr.toCharArray(tmpCharArr, tmpStr.length() + 1);
      if (debug) Serial.println(String(i) + ") " + tmpStr); // Вывести начальную строку
      GParser data2(tmpCharArr, ':'); // Парсим массив символов по символу запятой
      int am2 = data2.split(); // Получаем количество данных, внимание, ломает строку!
      if (am2 > 1) { // Если существует не только ключ, а ещё и значение
        String key = data2[0]; // Ключ - первое значение
        String value = data2[1]; // Значение - второе, или data.getInt(1), чтобы получить целое число
        if (debug) Serial.println("key: " + key + ", value: " + String(value)); // Вывод
        // Присваивание значений
        if (key.equals("kp")) pid.setKp(value.toFloat());
        else if (key.equals("ki")) pid.setKi(value.toFloat());
        else if (key.equals("kd")) pid.setKd(value.toFloat());
        else if (key.equals("speed")) speed = value.toInt();
      }
    }
    if (debug) Serial.println(); // Перевод на новую строку для разделения значений, которые были введены
  }
}