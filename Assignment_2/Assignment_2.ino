#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

const int digitalSignalPin = 14;
const int BUTTON_GPIO = 2;
const int LED_GPIO = 0;
const int SIGNAL_INPUT_1 = 22; // 用于接收第一个方波信号的GPIO引脚
const int SIGNAL_INPUT_2 = 23; // 用于接收第二个方波信号的GPIO引脚

#define MAX_READINGS 12
#define MAX_VOLTAGE 3.3
#define ANALOG_READ_PIN 26 // Arduino模拟读取引脚
#define LED_PIN 18 // LED连接的数字引脚
#define DEBOUNCE_DELAY pdMS_TO_TICKS(50)

SemaphoreHandle_t globalStructMutex;

float readings[MAX_READINGS] = {0.0}; // 存储最近10次读数的数组
int readIndex = 0; // 当前读数的索引
float total = 0; // 读数的总和
float average = 0; // 滑动平均值

SemaphoreHandle_t frequencyDataSemaphore;
QueueHandle_t buttonEventQueue;

typedef struct {
    int task2Frequency;
    int task3Frequency;
} FrequencyData;

FrequencyData globalFrequencyData;

//task1
void digitalSignalOutputTask(void *pvParameters) {
  // Configure the digital pin as an output
  pinMode(digitalSignalPin, OUTPUT);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFreequency = pdMS_TO_TICKS(4);

  // This task should never return, so it's enclosed in an infinite loop
  while (1) {
    // Turn the digital signal HIGH for 180μs
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(180);  // Directly delay for microseconds

    // Turn the signal LOW for 40μs
    digitalWrite(digitalSignalPin, LOW);
    delayMicroseconds(40);
    
    // Turn the signal HIGH again for 530μs
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(530);
    
    // Finally, turn the signal LOW for 3.25ms
    digitalWrite(digitalSignalPin, LOW);
    // delayMicroseconds(3250);
  }
}

//task2
void frequencyMeasurementTask1(void *parameters) {
    pinMode(SIGNAL_INPUT_1, INPUT);
    for (;;) {
        unsigned long duration = pulseIn(SIGNAL_INPUT_1, HIGH, 1000000); // 测量高电平持续时间（微秒）
        if (duration > 0) {
            int frequency = 1000000 / (2 * duration); // 计算频率
            xSemaphoreTake(frequencyDataSemaphore, portMAX_DELAY);
            globalFrequencyData.task2Frequency = frequency;
            xSemaphoreGive(frequencyDataSemaphore);
            // Serial.print("Task 2 Frequency: ");
            // Serial.println(frequency); // 打印频率值
        } else {
            // Serial.println("Task 2 Frequency: No signal");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 根据任务周期调整
    }
}
//task3
void frequencyMeasurementTask2(void *parameters) {
    pinMode(SIGNAL_INPUT_2, INPUT);
    for (;;) {
        unsigned long duration = pulseIn(SIGNAL_INPUT_2, HIGH, 1000000); // 测量高电平持续时间（微秒）
        if (duration > 0) {
            int frequency = 1000000 / (2 * duration); // 计算频率
            xSemaphoreTake(frequencyDataSemaphore, portMAX_DELAY);
            globalFrequencyData.task3Frequency = frequency;
            xSemaphoreGive(frequencyDataSemaphore);
            // Serial.print("Task 3 Frequency: ");
            // Serial.println(frequency); // 打印频率值
        } else {
            // Serial.println("Task 3 Frequency: No signal");
        }

        vTaskDelay(pdMS_TO_TICKS(8)); // 根据任务周期调整
    }
}


//task4
void sampleAnalogInputTask(void *parameters) {
  pinMode(LED_PIN, OUTPUT); // 设置LED引脚为输出模式
  for (;;) {
    // 读取模拟输入
    float analogValue = analogRead(ANALOG_READ_PIN) * (MAX_VOLTAGE / 4095.0); 
    total -= readings[readIndex]; // 移除最旧的读数
    readings[readIndex] = analogValue; // 添加最新的读数
    total += readings[readIndex]; // 更新总和
    readIndex = (readIndex + 1) % MAX_READINGS; // 更新索引

    if (readIndex == 0) { // 只有当数组填满时才更新平均值
      average = total / MAX_READINGS;
      // 检查是否需要显示错误
      if (average > (MAX_VOLTAGE / 2)) {
        digitalWrite(LED_PIN, HIGH); // 点亮LED
      } else {
        digitalWrite(LED_PIN, LOW); // 熄灭LED
      }

      // 输出当前模拟值和平均值
      Serial.print("Current analog value: ");
      Serial.print(analogValue);
      Serial.print(", Average: ");
      Serial.println(average);
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 根据任务周期调整延时
  }
}

//task5
void loggingTask(void *parameters) {
    for (;;) {
        // 尝试获取互斥量以安全访问全局结构体
        if (xSemaphoreTake(globalStructMutex, portMAX_DELAY) == pdTRUE) {
            // 缩放和限制频率值
            int scaledTask2Frequency = scaleAndBoundFrequency(globalFrequencyData.task2Frequency);
            int scaledTask3Frequency = scaleAndBoundFrequency(globalFrequencyData.task3Frequency);

            // 输出到串行端口
            Serial.printf("%d,%d\n", scaledTask2Frequency, scaledTask3Frequency);

            // 释放互斥量
            xSemaphoreGive(globalStructMutex);
        }

        // 按照任务要求延迟200毫秒
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int scaleAndBoundFrequency(int frequency) {
    // 计算缩放前先转换为浮点数以保留小数部分
    float scaledFrequency = (float)(frequency - 333) / (667) * 99;
    // 四舍五入结果，并确保结果在0到99之间
    int result = (int)(scaledFrequency + 0.5); // 加0.5后取整实现四舍五入
    result = max(0, min(result, 99)); // 保证值在0到99之间
    return result;
}

//task7
void buttonLEDControlTask(void *parameters) {
    pinMode(BUTTON_GPIO, INPUT_PULLUP); // 使用内部上拉电阻
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LOW); // 初始化LED状态为熄灭
    int lastButtonState = HIGH; // 初始状态为未按下
    int buttonState = lastButtonState;
    TickType_t lastDebounceTime = xTaskGetTickCount();

    for (;;) {
        int readButton = digitalRead(BUTTON_GPIO);
        
        // 检测按键状态是否有变化
        if (readButton != lastButtonState) {
            lastDebounceTime = xTaskGetTickCount();
        }

        // 检查是否超过去抖动阈值
        if ((xTaskGetTickCount() - lastDebounceTime) > DEBOUNCE_DELAY) {
            // 如果当前按键状态与上次检测的状态不同，则更新状态
            if (readButton != buttonState) {
                buttonState = readButton;
                // 如果按键被按下，则切换LED状态并输出消息
                if (buttonState == LOW) { // 假设按键按下时为低电平
                    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
                    Serial.println("button was pressed"); // 输出消息
                }
            }
        }

        lastButtonState = readButton;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
//task8
void CPU_work(int time) {
  // Serial.print("CPU_work start: ");
  // Serial.println(millis());
  volatile unsigned long endTime = millis() + time;
  while (millis() < endTime) {
    // 忙等待循环
  }
  // Serial.print("CPU_work end: ");
  // Serial.println(millis());
}

void periodicTask(void *pvParameters) {
  const TickType_t xFrequency = 20; // 20ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Serial.print("Task start: ");
    // Serial.println(millis());
    CPU_work(2); // 让CPU忙碌约2毫秒
    // Serial.print("Task end: ");
    // Serial.println(millis());
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency)); // 等待直到下一个周期
  }
}


void setup() {
  Serial.begin(9600);
  delay(1000);

  frequencyDataSemaphore = xSemaphoreCreateMutex();
  globalStructMutex = xSemaphoreCreateMutex();
  buttonEventQueue = xQueueCreate(10, sizeof(int));

  // 给数字信号输出任务最高优先级
  xTaskCreate(digitalSignalOutputTask, "DigitalSignalOutput", 2048, NULL, 2, NULL);
  // 给频率测量任务次高优先级
  xTaskCreate(frequencyMeasurementTask1, "FreqMeasure1", 2048, NULL, 3, NULL);
  xTaskCreate(frequencyMeasurementTask2, "FreqMeasure2", 2048, NULL, 3, NULL);
  // 给模拟输入采样任务稍低的高优先级
  xTaskCreate(sampleAnalogInputTask, "AnalogSample", 2048, NULL, 2, NULL);
  // 日志记录任务优先级较低
  xTaskCreate(loggingTask, "Logging", 2048, NULL, 1, NULL);
  // 按钮LED控制任务和周期性任务具有最低优先级
  xTaskCreate(buttonLEDControlTask, "ButtonLED", 2048, NULL, 0, NULL);
  xTaskCreate(periodicTask, "Periodic Task", 2048, NULL, 0, NULL); // 创建周期性任务
}


void loop() {
    // The loop function is intentionally left empty.
    // FreeRTOS tasks will handle all operations.
    vTaskDelay(pdMS_TO_TICKS(1000)); // Reduce idle spinning.
}
