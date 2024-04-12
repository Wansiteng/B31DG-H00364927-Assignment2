#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"

// Pin definitions for various signals and peripherals
const int digitalSignalPin = 14;
const int BUTTON_GPIO = 2;
const int LED_GPIO = 0;
const int SIGNAL_INPUT_1 = 22; 
const int SIGNAL_INPUT_2 = 23;

#define MAX_READINGS 12
#define MAX_VOLTAGE 3.3
#define ANALOG_READ_PIN 26
#define LED_PIN 18
#define DEBOUNCE_DELAY pdMS_TO_TICKS(50)

// Semaphores and queues for inter-task communication
SemaphoreHandle_t globalStructMutex;
float readings[MAX_READINGS] = {0.0};
int readIndex = 0;
float total = 0;
float average = 0;
SemaphoreHandle_t frequencyDataSemaphore;
QueueHandle_t buttonEventQueue;

// Struct to hold frequency data shared between tasks
typedef struct {
    int task2Frequency;
    int task3Frequency;
} FrequencyData;
FrequencyData globalFrequencyData;

// Task to output a digital signal at specified intervals
void digitalSignalOutputTask(void *pvParameters) {
  pinMode(digitalSignalPin, OUTPUT);
  while (1) {
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(180);
    digitalWrite(digitalSignalPin, LOW);
    delayMicroseconds(40);
    digitalWrite(digitalSignalPin, HIGH);
    delayMicroseconds(530);
    digitalWrite(digitalSignalPin, LOW);
  }
}

// Task to measure frequency from the first signal input
void frequencyMeasurementTask1(void *parameters) {
    pinMode(SIGNAL_INPUT_1, INPUT);
    for (;;) {
        unsigned long duration = pulseIn(SIGNAL_INPUT_1, HIGH, 1000000);
        if (duration > 0) {
            int frequency = 1000000 / (2 * duration);
            xSemaphoreTake(frequencyDataSemaphore, portMAX_DELAY);
            globalFrequencyData.task2Frequency = frequency;
            xSemaphoreGive(frequencyDataSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task to measure frequency from the second signal input
void frequencyMeasurementTask2(void *parameters) {
    pinMode(SIGNAL_INPUT_2, INPUT);
    for (;;) {
        unsigned long duration = pulseIn(SIGNAL_INPUT_2, HIGH, 1000000);
        if (duration > 0) {
            int frequency = 1000000 / (2 * duration);
            xSemaphoreTake(frequencyDataSemaphore, portMAX_DELAY);
            globalFrequencyData.task3Frequency = frequency;
            xSemaphoreGive(frequencyDataSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

// Task to sample an analog input and calculate a moving average
void sampleAnalogInputTask(void *parameters) {
  pinMode(LED_PIN, OUTPUT);
  for (;;) {
    float analogValue = analogRead(ANALOG_READ_PIN) * (MAX_VOLTAGE / 4095.0);
    total -= readings[readIndex];
    readings[readIndex] = analogValue;
    total += readings[readIndex];
    readIndex = (readIndex + 1) % MAX_READINGS;
    if (readIndex == 0) {
      average = total / MAX_READINGS;
      digitalWrite(LED_PIN, average > (MAX_VOLTAGE / 2) ? HIGH : LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Task for logging frequency measurements to serial
void loggingTask(void *parameters) {
    for (;;) {
        if (xSemaphoreTake(globalStructMutex, portMAX_DELAY) == pdTRUE) {
            int scaledTask2Frequency = scaleAndBoundFrequency(globalFrequencyData.task2Frequency);
            int scaledTask3Frequency = scaleAndBoundFrequency(globalFrequencyData.task3Frequency);
            Serial.printf("%d,%d\n", scaledTask2Frequency, scaledTask3Frequency);
            xSemaphoreGive(globalStructMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Utility function to scale frequency measurements
int scaleAndBoundFrequency(int frequency) {
    float scaledFrequency = (float)(frequency - 333) / 667 * 99;
    int result = (int)(scaledFrequency + 0.5);
    return max(0, min(result, 99));
}

// Task to control LED state based on button input with debounce
void buttonLEDControlTask(void *parameters) {
    pinMode(BUTTON_GPIO, INPUT_PULLUP);
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LOW);
    int lastButtonState = HIGH;
    int buttonState = lastButtonState;
    TickType_t lastDebounceTime = xTaskGetTickCount();
    for (;;) {
        int readButton = digitalRead(BUTTON_GPIO);
        if (readButton != lastButtonState) {
            lastDebounceTime = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - lastDebounceTime) > DEBOUNCE_DELAY) {
            if (readButton != buttonState) {
                buttonState = readButton;
                if (buttonState == LOW) {
                    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
                    Serial.println("button was pressed");
                }
            }
        }
        lastButtonState = readButton;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task to simulate CPU workload for a specified duration
void CPU_work(int time) {
  volatile unsigned long endTime = millis() + time;
  while (millis() < endTime) { }
}

// Periodic task to manage timing and workload simulation
void periodicTask(void *pvParameters) {
  const TickType_t xFrequency = 20; // Task frequency in ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    CPU_work(2); // Simulate 2 ms CPU work
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  frequencyDataSemaphore = xSemaphoreCreateMutex();
  globalStructMutex = xSemaphoreCreateMutex();
  buttonEventQueue = xQueueCreate(10, sizeof(int));
  // Assigning priorities to tasks based on their importance
  xTaskCreate(digitalSignalOutputTask, "DigitalSignalOutput", 2048, NULL, 2, NULL);
  xTaskCreate(frequencyMeasurementTask1, "FreqMeasure1", 2048, NULL, 3, NULL);
  xTaskCreate(frequencyMeasurementTask2, "FreqMeasure2", 2048, NULL, 3, NULL);
  xTaskCreate(sampleAnalogInputTask, "AnalogSample", 2048, NULL, 2, NULL);
  xTaskCreate(loggingTask, "Logging", 2048, NULL, 1, NULL);
  xTaskCreate(buttonLEDControlTask, "ButtonLED", 2048, NULL, 0, NULL);
  xTaskCreate(periodicTask, "Periodic Task", 2048, NULL, 0, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Reduce idle spinning.
}
