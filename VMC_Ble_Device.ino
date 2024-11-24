
/*
  实际电压 分压后的电压  ADC值
  3.7 | 1.224 | 1325
  3.8 | 1.260 | 1368
  3.9 | 1.292 | 1408
  4.0 | 1.323 | 1445
  4.1 | 1.356 | 1486
  4.2 | 1.391 | 1528
*/
#include <Arduino.h>
#include <BleGamepad.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>

#define USE_NIMBLE

// 启用的按钮
#define numOfButtons 18
#define numOfHatSwitches 0
#define enableX true
#define enableY true
#define enableZ false
#define enableRX true
#define enableRY true
#define enableRZ false
#define enableSlider1 false
#define enableSlider2 false
#define enableRudder false
#define enableThrottle false
#define enableAccelerator false
#define enableBrake false
#define enableSteering false
// 电池电量读取相关配置

#define BATTERY_ADC_READING_TIMES 100            // ADC读取电池电压次数
#define V_REF 3.3                                // 实测系统电压
#define BATTERY_ADC_READING_CYCLE 5 * 60 * 1000  // 5分钟

// 蓝牙HID设备参数
#define VID 0x3412
#define PID 0x6500

// 按键矩阵大小
#define ROWS 4
#define COLS 4

#define KEYPAD_KEY_NUM 16

#define BUTTON_JOYSTICK1 KEYPAD_KEY_NUM + 1
#define BUTTON_JOYSTICK2 KEYPAD_KEY_NUM + 2

#define NUMPIXELS 8    // WS2812-8 LED有8个LED
#define BRIGHTNESS 20  // 设置亮度，范围从 0 到 255

#define JOYSTICK_ADC_SAMPLE_NUM 20
#define JOYSTICK_ADC_SAMPLE_DELAY 1
#define MIN_ADC_VALUE 0
#define MAX_ADC_VALUE 4095

// 引脚定义
#define BATTERY_PIN 36  // 电池引脚

#define JOYSTICK1_VRX_PIN 39
#define JOYSTICK1_VRY_PIN 34
#define JOYSTICK1_SW_PIN 4

#define JOYSTICK2_VRX_PIN 35
#define JOYSTICK2_VRY_PIN 32
#define JOYSTICK2_SW_PIN 0

#define ROW_PIN_1 22
#define ROW_PIN_2 33
#define ROW_PIN_3 25
#define ROW_PIN_4 26

#define COL_PIN_1 27
#define COL_PIN_2 14
#define COL_PIN_3 12
#define COL_PIN_4 13

#define LED_STRIP_PIN 16

typedef enum {
  cmd_unknown,
  cmd_set_led_show,
  cmd_set_led_color_show,
  cmd_set_led_color,
  cmd_set_led_brightness_show,
  cmd_set_led_brightness,
  cmd_set_led_clear_show,
  cmd_set_led_clear,
  cmd_set_led_fill_show,
  cmd_set_led_fill,
  cmd_set_joystick_deadzone,
  cmd_set_joystick_max_value,

} CommandTypeDef;



// 初始化BLE
BleGamepad bleGamepad("VMC BLE Gamepad", "Sab1e", 100);
BleGamepadConfiguration bleGamepadConfig;
Adafruit_NeoPixel strip(NUMPIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// 电池电量检测提供的校准数据
const float voltages[] = { 3.7, 3.8, 3.9, 4.0, 4.1, 4.2 };
const int adcValues[] = { 1325, 1368, 1408, 1445, 1486, 1528 };
const int numPoints = sizeof(voltages) / sizeof(voltages[0]);

// 按键矩阵
uint8_t rowPins[ROWS] = { ROW_PIN_1, ROW_PIN_2, ROW_PIN_3, ROW_PIN_4 };
uint8_t colPins[COLS] = { COL_PIN_1, COL_PIN_2, COL_PIN_3, COL_PIN_4 };
uint8_t keymap[ROWS][COLS] = {
  { 1, 2, 3, 4 },     // Buttons  1,  2,  3,  4      --> Used for calulating the bitmask for sending to the library
  { 5, 6, 7, 8 },     // Buttons  5,  6,  7,  8      --> Adjust to suit which buttons you want the library to send
  { 9, 10, 11, 12 },  // Buttons  9, 10, 11, 12      -->
  { 13, 14, 15, 16 }  // Buttons 13, 14, 15, 16      --> Eg. The value 12 in the array refers to button 12
};

// (0~32767)
uint16_t joystick1Deadzone = 0;
uint16_t joystick2Deadzone = 0;

uint16_t joystick1MaxValue = 30000;
uint16_t joystick2MaxValue = 30000;

bool isPlayingSystemAnimation = false;  //如果当前正在播放系统动画，就禁止上位机控制灯光。

Keypad customKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, ROWS, COLS);

uint8_t getBatteryPercentage() {
  uint32_t adcValue = 0;

  // 多次读取电池电压并取平均值
  for (int i = 0; i < BATTERY_ADC_READING_TIMES; i++) {
    adcValue += analogRead(BATTERY_PIN);
    vTaskDelay(100);
  }

  adcValue /= BATTERY_ADC_READING_TIMES;

  // 如果ADC值低于最低点，直接返回0
  if (adcValue <= adcValues[0]) {
    return 0;
  }
  // 如果ADC值高于最高点，直接返回100
  if (adcValue >= adcValues[numPoints - 1]) {
    return 100;
  }

  // 在线性区间内查找电池电压
  float batteryVoltage = 0.0;
  for (int i = 0; i < numPoints - 1; i++) {
    if (adcValue >= adcValues[i] && adcValue <= adcValues[i + 1]) {
      // 线性插值公式
      batteryVoltage = voltages[i] + (voltages[i + 1] - voltages[i]) * (adcValue - adcValues[i]) / (adcValues[i + 1] - adcValues[i]);
      break;
    }
  }

  // 根据电压计算电池百分比
  uint8_t batteryPercentage = map(batteryVoltage * 1000, 3700, 4200, 0, 100);

  // 限制百分比在0-100之间
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  return batteryPercentage;
}
int16_t getJoystickAxesValue(uint16_t pin) {
  int adcValues[JOYSTICK_ADC_SAMPLE_NUM];

  // 读取多个ADC值
  for (int i = 0; i < JOYSTICK_ADC_SAMPLE_NUM; i++) {
    adcValues[i] = analogRead(pin);
    vTaskDelay(JOYSTICK_ADC_SAMPLE_DELAY);
  }

  // 使用中值滤波去噪
  int16_t medianValue = median(adcValues, JOYSTICK_ADC_SAMPLE_NUM);

  // 将ADC值映射到-32767到32767范围
  int16_t value = map(medianValue, 0, 4095, -32767, 32767);
  
  return value;
}

// 中值函数实现
int16_t median(int* arr, int size) {
  // 复制数组
  int temp[size];
  memcpy(temp, arr, size * sizeof(int));

  // 使用简单的选择排序对数组进行排序
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (temp[i] > temp[j]) {
        int tmp = temp[i];
        temp[i] = temp[j];
        temp[j] = tmp;
      }
    }
  }

  // 返回中位数
  return temp[size / 2];
}





void KeypadUpdate() {
  customKeypad.getKeys();

  for (int i = 0; i < LIST_MAX; i++)  // Scan the whole key list.      //LIST_MAX is provided by the Keypad library and gives the number of buttons of the Keypad instance
  {
    if (customKeypad.key[i].stateChanged)  // Only find keys that have changed state.
    {
      uint8_t keystate = customKeypad.key[i].kstate;

      if (bleGamepad.isConnected()) {
        if (keystate == PRESSED) {
          bleGamepad.press(customKeypad.key[i].kchar);
        }  // Press or release button based on the current state
        if (keystate == RELEASED) {
          bleGamepad.release(customKeypad.key[i].kchar);
        }

        // bleGamepad.sendReport();  // Send the HID report after values for all button states are updated, and at least one button state had changed
      }
    }
  }
}


//===================FreeRTOS Tasks===================
void BatteryLevelDetectionTask(void *pvParameters) {
  while (1) {
    uint8_t batteryPercentage = getBatteryPercentage();
    Serial.print("Battery Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("");
    if (bleGamepad.isConnected()) {
      bleGamepad.setBatteryLevel(batteryPercentage);
    }
    if (batteryPercentage <= 5) {
      isPlayingSystemAnimation = true;
      vTaskDelay(300);
      BlinkLEDStrip(3);
      vTaskDelay(300);
      isPlayingSystemAnimation = false;
    }
    vTaskDelay(BATTERY_ADC_READING_CYCLE);
  }
}

void ScanKeypadTask(void *pvParameters) {
  while (1) {
    if (bleGamepad.isConnected()) {
      // 发送按键矩阵数据
      KeypadUpdate();
    }
    vTaskDelay(100);
    bleGamepad.sendReport();
  }
}
// Arduino代码中获取摇杆输入值的函数
int16_t mapJoystickValue(int16_t rawValue, int16_t deadzone, int16_t maxValue) {
  // 死区处理: 如果原始值在死区范围内，输出为0
  if (abs(rawValue) < deadzone) {
    return 0;
  }

  // 限制最大值和最小值
  if (rawValue > maxValue) {
    rawValue = maxValue;
  } else if (rawValue < -maxValue) {
    rawValue = -maxValue;
  }

  // 映射：将值从 [deadzone, maxValue] 映射到 [-32767, 32767] 范围
  // 假设 `rawValue` 已经大于死区，按比例映射
  if (rawValue > 0) {
    return map(rawValue, deadzone, maxValue, 0, 32767);
  } else {
    return map(rawValue, -maxValue, -deadzone, -32767, 0);
  }
}

void HandleJoystick1Task(void *pvParameters) {
  while (1) {
    if (bleGamepad.isConnected()) {
      // 使用映射函数处理X和Y轴的值
      int16_t mappedX = mapJoystickValue(getJoystickAxesValue(JOYSTICK1_VRX_PIN), joystick1Deadzone, joystick1MaxValue);
      int16_t mappedY = mapJoystickValue(getJoystickAxesValue(JOYSTICK1_VRY_PIN), joystick1Deadzone, joystick1MaxValue);

      // 设置游戏手柄的X、Y轴值
      bleGamepad.setX(mappedX);
      bleGamepad.setY(mappedY);

      // 按钮状态判断
      if (digitalRead(JOYSTICK1_SW_PIN) == LOW) {
        bleGamepad.press(BUTTON_JOYSTICK1);
      } else {
        bleGamepad.release(BUTTON_JOYSTICK1);
      }
    }
    // vTaskDelay(50);
  }
}

void HandleJoystick2Task(void *pvParameters) {
  while (1) {
    if (bleGamepad.isConnected()) {
      // 使用映射函数处理X和Y轴的值
      int16_t mappedX = mapJoystickValue(getJoystickAxesValue(JOYSTICK2_VRX_PIN), joystick2Deadzone, joystick2MaxValue);
      int16_t mappedY = mapJoystickValue(getJoystickAxesValue(JOYSTICK2_VRY_PIN), joystick2Deadzone, joystick2MaxValue);

      // 设置游戏手柄的X、Y轴值
      bleGamepad.setRX(mappedX);
      bleGamepad.setRY(mappedY);

      // 按钮状态判断
      if (digitalRead(JOYSTICK2_SW_PIN) == LOW) {
        bleGamepad.press(BUTTON_JOYSTICK2);
      } else {
        bleGamepad.release(BUTTON_JOYSTICK2);
      }
    }
    // vTaskDelay(50);
  }
}




void HandleOutputReportTask(void *pvParameters) {
  /*
  Output协议：
  字节0 - 命令
  其余字节 - 参数

  LED 灯带协议：

  cmd_set_led_show
  更新颜色
  无参数

  cmd_set_led_color_show
  修改颜色并立即更新
  字节1 - LED索引
  字节2 - 颜色R
  字节3 - 颜色G
  字节4 - 颜色B

  cmd_set_led_color
  修改颜色
  字节1 - LED索引
  字节2 - 颜色R
  字节3 - 颜色G
  字节4 - 颜色B

  cmd_set_led_brightness_show
  修改亮度并立即更新
  字节1 - 亮度

  cmd_set_led_brightness
  修改亮度
  字节1 - 亮度

  cmd_set_led_clear_show
  清空所有亮度并立即更新

  cmd_set_led_clear
  清空所有亮度

  cmd_set_led_fill_show
  填充多个LED颜色并立即更新
  字节1 - 开始填充LED索引
  字节2 - 填充LED的数量
  字节3 - 颜色R
  字节4 - 颜色G
  字节5 - 颜色B

  cmd_set_led_fill
  填充多个LED颜色
  字节1 - 开始填充LED索引
  字节2 - 填充LED的数量
  字节3 - 颜色R
  字节4 - 颜色G
  字节5 - 颜色B

  cmd_set_led_color_all 弃用
  使用同一个包填充多个LED颜色
  LED Index 0
  字节1 - 颜色R
  字节2 - 颜色G
  字节3 - 颜色B
  LED Index 1
  字节3 - 颜色R
  字节4 - 颜色G
  字节5 - 颜色B
  ...
  LED Index 7
  字节22 - 颜色R
  字节23 - 颜色G
  字节24 - 颜色B

  cmd_set_joystick_deadzone_x
  设置摇杆x轴死区
  字节1 - 摇杆索引 {1,2}
  字节2,3 - 死区值(int16_t)

  cmd_set_joystick_deadzone_y
  设置摇杆y轴死区
  字节1 - 摇杆索引 {1,2}
  字节2,3 - 死区值(int16_t)

  cmd_set_joystick_max_value_x
  设置摇杆x轴最大值
  字节1 - 摇杆索引 {1,2}
  字节2,3 - 最大值(int16_t)

  cmd_set_joystick_max_value_y
  设置摇杆y轴最大值
  字节1 - 摇杆索引 {1,2}
  字节2,3 - 最大值(int16_t)

*/
  while (1) {
    if (bleGamepad.isConnected()) {
      if (bleGamepad.isOutputReceived()) {
        if (!isPlayingSystemAnimation) {
          uint8_t *buffer = bleGamepad.getOutputBuffer();
          switch (buffer[0]) {
            case cmd_set_led_show:
              strip.show();
              break;
            case cmd_set_led_color_show:
              strip.setPixelColor(buffer[1], strip.Color(buffer[2], buffer[3], buffer[4]));
              strip.show();
              break;
            case cmd_set_led_color:
              strip.setPixelColor(buffer[1], strip.Color(buffer[2], buffer[3], buffer[4]));
              break;
            case cmd_set_led_brightness_show:
              strip.setBrightness(buffer[1]);
              strip.show();
              break;
            case cmd_set_led_brightness:
              strip.setBrightness(buffer[1]);
              break;
            case cmd_set_led_clear_show:
              strip.clear();
              strip.show();
              break;
            case cmd_set_led_clear:
              strip.clear();
              break;
            case cmd_set_led_fill_show:
              strip.fill(strip.Color(buffer[3], buffer[4], buffer[5]), buffer[1], buffer[2]);
              strip.show();
              break;
            case cmd_set_led_fill:
              strip.fill(strip.Color(buffer[3], buffer[4], buffer[5]), buffer[1], buffer[2]);
              break;
            // case cmd_set_led_color_all_show:
            //   strip.setPixelColor(0, strip.Color(buffer[1], buffer[2], buffer[3]));
            //   strip.setPixelColor(1, strip.Color(buffer[4], buffer[5], buffer[6]));
            //   strip.setPixelColor(2, strip.Color(buffer[7], buffer[8], buffer[9]));
            //   strip.setPixelColor(3, strip.Color(buffer[10], buffer[11], buffer[12]));
            //   strip.setPixelColor(4, strip.Color(buffer[13], buffer[14], buffer[15]));
            //   strip.setPixelColor(5, strip.Color(buffer[16], buffer[17], buffer[18]));
            //   strip.setPixelColor(6, strip.Color(buffer[19], buffer[20], buffer[21]));
            //   strip.setPixelColor(7, strip.Color(buffer[22], buffer[23], buffer[24]));
            //   strip.show();
              // break;
            case cmd_set_joystick_deadzone:
              if(buffer[1]==1){
                joystick1Deadzone = (uint16_t)((buffer[3] << 8) | buffer[2]);
              }else if(buffer[1]==2){
                joystick2Deadzone = (uint16_t)((buffer[3] << 8) | buffer[2]);
              }
              break;
            case cmd_set_joystick_max_value:
              if(buffer[1]==1){
                joystick1MaxValue = (uint16_t)((buffer[3] << 8) | buffer[2]);
              }else if(buffer[1]==2){
                joystick2MaxValue = (uint16_t)((buffer[3] << 8) | buffer[2]);
              }
              break;
              break;
            default:
              break;
          }
        }
      }
    }
  }
}
//===================FreeRTOS Tasks===================


//===================LED Strip Effects===================
void LEDStripClearAll() {
  strip.clear();  //关闭所有LED
  strip.show();
}
void PlayLEDStripLoadingAnimation(uint8_t red, uint8_t green, uint8_t blue) {
  LEDStripClearAll();
  const uint8_t stripLength = 3;

  // 从左到右加载
  for (int i = 0; i < NUMPIXELS + stripLength; i++) {
    if (i < NUMPIXELS) {
      strip.setPixelColor(i, strip.Color(red, green, blue));
    }

    if (i >= stripLength) {
      strip.setPixelColor(i - stripLength, strip.Color(0, 0, 0));
    }

    strip.show();
    vTaskDelay(100);
  }

  vTaskDelay(500);

  // 从右到左加载
  for (int i = 0; i < NUMPIXELS + stripLength; i++) {
    if (i < NUMPIXELS) {
      strip.setPixelColor(NUMPIXELS - 1 - i, strip.Color(red, green, blue));
    }

    if (i >= stripLength) {
      strip.setPixelColor(NUMPIXELS - 1 - (i - stripLength), strip.Color(0, 0, 0));
    }

    strip.show();
    vTaskDelay(100);
  }
}

// 播放LED灯带的呼吸动画
void PlayLEDStripBreathAnimation(uint8_t red, uint8_t green, uint8_t blue) {
  LEDStripClearAll();  // 先清空所有LED

  // 定义渐变的步进值和呼吸的延时
  uint8_t brightness = 0;  // 初始亮度
  int fadeAmount = 5;      // 每次亮度变化的增量
  int delayTime = 20;      // 每步延时（控制呼吸速度）

  // 呼吸效果：逐渐增加亮度
  while (brightness < 255) {
    // 设置所有LED的颜色和当前亮度
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(
                               red * brightness / 255,    // 调整红色亮度
                               green * brightness / 255,  // 调整绿色亮度
                               blue * brightness / 255    // 调整蓝色亮度
                               ));
    }
    strip.show();  // 更新LED显示

    // 增加亮度
    brightness += fadeAmount;

    // 控制渐变速度
    vTaskDelay(delayTime);  // 延时控制呼吸效果的速度
  }

  // 呼吸效果：逐渐减小亮度
  while (brightness > 0) {
    // 设置所有LED的颜色和当前亮度
    for (int i = 0; i < NUMPIXELS; i++) {
      strip.setPixelColor(i, strip.Color(
                               red * brightness / 255,    // 调整红色亮度
                               green * brightness / 255,  // 调整绿色亮度
                               blue * brightness / 255    // 调整蓝色亮度
                               ));
    }
    strip.show();  // 更新LED显示

    // 减少亮度
    brightness -= fadeAmount;

    // 控制渐变速度
    vTaskDelay(delayTime);  // 延时控制呼吸效果的速度
  }

  // 完成呼吸效果后关闭LED
  LEDStripClearAll();
}

void PlayLEDStripPowerOnAnimation() {
  uint8_t color[NUMPIXELS][3] = {
    { 0, 255, 0 },
    { 120, 255, 0 },
    { 200, 255, 0 },
    { 255, 255, 0 },
    { 255, 160, 0 },
    { 255, 80, 0 },
    { 255, 40, 0 },
    { 255, 0, 0 }
  };

  LEDStripClearAll();

  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(color[i][0], color[i][1], color[i][2]));
    strip.show();
    vTaskDelay(100);
  }
  vTaskDelay(200);
  for (int i = NUMPIXELS - 1; i >= 0; i--) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    vTaskDelay(100);
  }
}
void BlinkLEDStrip(uint8_t times) {
  for (int i = 0; i < times; i++) {
    strip.fill(strip.Color(255, 0, 0), 0, NUMPIXELS);
    strip.show();
    vTaskDelay(300);
    LEDStripClearAll();
    vTaskDelay(300);
  }
}
//===================LED Strip Effects===================
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Init VMC Device...");

  // 可选：设置是否包含其他按钮
  // bleGamepadConfig.setIncludeStart(true);   // 包括 Start 按钮
  // bleGamepadConfig.setIncludeSelect(true);  // 包括 Select 按钮
  // bleGamepadConfig.setIncludeHome(true);    // 包括 Home 按钮
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setEnableOutputReport(true);
  bleGamepadConfig.setOutputReportLength(6);
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setWhichAxes(enableX, enableY, enableZ, enableRX, enableRY, enableRZ, enableSlider1, enableSlider2);       // Can also be done per-axis individually. All are true by default
  bleGamepadConfig.setWhichSimulationControls(enableRudder, enableThrottle, enableAccelerator, enableBrake, enableSteering);  // Can also be done per-control individually. All are false by default
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);


  bleGamepadConfig.setVid(VID);
  bleGamepadConfig.setPid(PID);

  bleGamepadConfig.setAxesMin(0x8001);
  bleGamepadConfig.setAxesMax(0x7FFF);

  bleGamepad.begin(&bleGamepadConfig);
  // Serial.println("BLE Init Successful!");
  analogReadResolution(12);  // 12位ADC分辨率，范围是0~4095
  // Serial.println("ADC Init Successful!");

  // Init Pins
  pinMode(JOYSTICK1_SW_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK2_SW_PIN, INPUT_PULLUP);

  // Serial.println("Pins Init Successful!");

  // Init LED Strip
  strip.begin();                    // 初始化LED
  strip.setBrightness(BRIGHTNESS);  // 设置亮度
  strip.show();                     // 更新LED显示
  // Serial.println("LED Strip Init Successful!");

  // Create FreeRTOS Task
  xTaskCreate(BatteryLevelDetectionTask, "BatteryLevelDetectionTask", 10000, NULL, 1, NULL);
  xTaskCreate(ScanKeypadTask, "ScanKeypadTask", 10000, NULL, 1, NULL);
  xTaskCreate(HandleJoystick1Task, "HandleJoystick1Task", 10000, NULL, 1, NULL);
  xTaskCreate(HandleJoystick2Task, "HandleJoystick2Task", 10000, NULL, 1, NULL);
  xTaskCreate(HandleOutputReportTask, "HandleOutputReportTask", 10000, NULL, 1, NULL);
  // Serial.println("Task Create Successful!");


  PlayLEDStripPowerOnAnimation();

  Serial.println("VMC Device Init Successful!");
}

void loop() {
  // // RGB幻彩效果：通过不同的RGB组合改变颜色
  // for (int j = 0; j < 256; j++) { // 外层循环：RGB颜色变化的步进
  //   for (int i = 0; i < NUMPIXELS; i++) {
  //     // 计算平滑过渡的颜色
  //     uint8_t red = (i * 255 / NUMPIXELS + j) % 256;   // 红色渐变
  //     uint8_t green = (i * 255 / NUMPIXELS + j + 85) % 256; // 绿色渐变
  //     uint8_t blue = (i * 255 / NUMPIXELS + j + 170) % 256; // 蓝色渐变

  //     // 设置每个LED的颜色
  //     // strip.setPixelColor(i, strip.Color(red, green, blue));
  //   }
  //   // strip.show();        // 更新显示
  //   vTaskDelay(2000);           // 延时控制幻彩效果的平滑度
  // }
  if (!bleGamepad.isConnected()) {
    while (!bleGamepad.isConnected()) {
      PlayLEDStripLoadingAnimation(0, 0, 255);
      vTaskDelay(500);
    }
    PlayLEDStripBreathAnimation(0, 0, 255);
    bleGamepad.setBatteryLevel(getBatteryPercentage());
  }
  vTaskDelay(500);
}
