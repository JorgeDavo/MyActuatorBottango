#include "BottangoArduinoCallbacks.h"
#include "src/AbstractEffector.h"
#include "src/Log.h"
#include "src/Outgoing.h"
#include <SPI.h>
#include "test.h"
#include <Adafruit_NeoPixel.h>
#define PIN 4
#define NUM_LEDS 1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

CAN_device_t CAN_cfg;              // CAN Config
unsigned long previousMillis = 0;  // will store last time a CAN Message was send
unsigned long previousMillis1 = 0; // will store last time a CAN Message was send
unsigned long currentMillis = millis();
const int interval = 0;       // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10; // Receive Queue size

namespace Callbacks
{
  void sendMotorCommand(int32_t angleControl, uint16_t maxSpeed, int id)
  {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = id;                                 // CAN ID
    tx_frame.FIR.B.DLC = 8;                              // Data length
    tx_frame.data.u8[0] = 0xA4;                          // Command byte
    tx_frame.data.u8[1] = 0x00;                          // NULL
    tx_frame.data.u8[2] = (uint8_t)(maxSpeed);           // Speed limit low byte
    tx_frame.data.u8[3] = (uint8_t)(maxSpeed >> 8);      // Speed limit high byte
    tx_frame.data.u8[4] = (uint8_t)(angleControl);       // Position control low byte
    tx_frame.data.u8[5] = (uint8_t)(angleControl >> 8);  // Position control
    tx_frame.data.u8[6] = (uint8_t)(angleControl >> 16); // Position control
    tx_frame.data.u8[7] = (uint8_t)(angleControl >> 24); // Position control high byte

    if (ESP32Can.CANWriteFrame(&tx_frame))
    {
      Serial.println("Command sent successfully");
    }
    else
    {
      Serial.println("Failed to send command");
    }
  }

  void sendCustomMotionControlCommand(MotionControlData data)
  {
    // Parameters for motion control 運動控制参数
    float v_des = 0; // Desired velocity 期望速度
    float t_ff = 0;  // Feedforward torque 前馈扭矩

    // Adapt control parameters based on type and signal

    uint16_t p_des_packed = (uint16_t)(data.p_des_position + 32767.5);
    // uint16_t p_des_packed = (uint16_t)(data.p_des_position + 29350);

    uint16_t v_des_packed = (uint16_t)(((v_des + 45) / 90.0) * 4095);
    uint16_t t_ff_packed = (uint16_t)(((t_ff + 24) / 48.0) * 4095);
    uint16_t kp_packed = (uint16_t)((data.kp / 500.0) * 4095);
    uint16_t kd_packed = (uint16_t)((data.kd / 5.0) * 4095);

    // Preparing the CAN message buffer 准备CAN消息缓冲区

    CAN_frame_t rx_frame;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = data.id;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = p_des_packed >> 8;
    tx_frame.data.u8[1] = p_des_packed & 0xFF;
    tx_frame.data.u8[2] = v_des_packed >> 4;
    tx_frame.data.u8[3] = ((v_des_packed & 0xF) << 4) | (kp_packed >> 8);
    tx_frame.data.u8[4] = kp_packed & 0xFF;
    tx_frame.data.u8[5] = kd_packed >> 4;
    tx_frame.data.u8[6] = ((kd_packed & 0xF) << 4) | (t_ff_packed >> 8);
    tx_frame.data.u8[7] = t_ff_packed & 0xFF;
    ESP32Can.CANWriteFrame(&tx_frame);

    // Sending the CAN message 发送CAN消息
    // CAN.sendMsgBuf(0x400 + 1, 0, 8, buffer); // Send command with a predefined ID (0x401) 使用预定义的ID（0x401）发送命令
  }

  QueueHandle_t signalQueue;
  void sendCommandTask(void *parameter)
  {
    MotionControlData data; // Use MotionControlData to receive data
    while (1)
    {
      if (xQueueReceive(signalQueue, &data, portMAX_DELAY))
      {
        sendCustomMotionControlCommand(data);
      }
    }
  }

  void onThisControllerStarted()
  {
    strip.begin(); // Initialize the strip
    strip.show();  // Initialize all pixels to 'off'
    Motor1_Stop();
    // Set the first LED to red during startup
    strip.setPixelColor(0, strip.Color(20, 0, 0)); // Red color
    strip.show();                                  // Update the strip to show the color
    signalQueue = xQueueCreate(10, sizeof(MotionControlData));

    // Create the task
    xTaskCreatePinnedToCore(
        sendCommandTask,   /* Function to implement the task */
        "sendCommandTask", /* Name of the task */
        10000,             /* Stack size in words */
        NULL,              /* Task input parameter */
        0,                 /* Priority of the task */
        NULL,              /* Task handle. */
        1);

    esp_log_level_set("*", ESP_LOG_NONE);

    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    // Serial.begin(115200);
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    // Init CAN Module
    ESP32Can.CANInit();
    //delay(10);
    //Motor1_Stop();
    //Motor2_Stop();
    //delay(10);
    //sendMotorCommand(1, 20, 0x141);
    //sendMotorCommand(1, 20, 0x142);
    //delay(7000);
    //Motor1_Stop();
    //Motor2_Stop();
    delay(10);
    strip.setPixelColor(0, strip.Color(0, 20, 0)); // Green color
    strip.show();                                  // Update the strip to show the color
  }

  void onThisControllerStopped()
  {
    strip.setPixelColor(0, strip.Color(20, 0, 0)); // Red color
    strip.show();                                  // Update the strip to show the color
  }

  void onEarlyLoop()
  {
    unsigned long currentMillis = millis();
  }

  void onLateLoop()
  {
    delayMicroseconds(100);
  }

  void onEffectorRegistered(AbstractEffector *effector)
  {
  }

  void effectorSignalOnLoop(AbstractEffector *effector, int signal)
  {
    currentMillis = millis(); // Update currentMillis with the current time
    char effectorIdentifier[9];
    effector->getIdentifier(effectorIdentifier, 9);
    MotionControlData data;

    if (strcmp(effectorIdentifier, "1") == 0)
    {
      //------------------------------------------------EYE UP AND DOWN
      if (currentMillis - previousMillis >= interval)
      {
        data.p_des_position = signal; // Assuming 'position' is available in this scope
        data.id = 0x401;              // Adapt this based on your needs
        data.kp = 20;                 // Adapt this based on your needs45
        data.kd = .4;                 // Adapt this based on your needs3
        xQueueSend(signalQueue, &data, portMAX_DELAY);
        previousMillis = currentMillis; // Update time for the last command sent
      }
    }
    if (strcmp(effectorIdentifier, "2") == 0)
    {
      if (currentMillis - previousMillis1 >= interval)
      {
        data.p_des_position = signal; // Assuming 'position' is available in this scope
        data.id = 0x402;              // Adapt this based on your needs
        data.kp = 20;                 // Adapt this based on your needs
        data.kd = .15;                // Adapt this based on your needs
        xQueueSend(signalQueue, &data, portMAX_DELAY);
        previousMillis1 = currentMillis;
      }
    }
    if (strcmp(effectorIdentifier, "3") == 0)
    {
      data.p_des_position = signal; // Assuming 'position' is available in this scope
      data.id = 0x403;              // Adapt this based on your needs
      data.kp = 100;                // Adapt this based on your needs
      data.kd = 2;                  // Adapt this based on your needs
      xQueueSend(signalQueue, &data, portMAX_DELAY);
    }
    if (strcmp(effectorIdentifier, "4") == 0)
    {
      data.p_des_position = signal; // Assuming 'position' is available in this scope
      data.id = 0x404;              // Adapt this based on your needs
      data.kp = 20;                 // Adapt this based on your needs
      data.kd = 0.15;               // Adapt this based on your needs
      xQueueSend(signalQueue, &data, portMAX_DELAY);
    }
    if (strcmp(effectorIdentifier, "5") == 0)
    {
      data.p_des_position = signal; // Assuming 'position' is available in this scope
      data.id = 0x405;              // Adapt this based on your needs
      data.kp = 20;                 // Adapt this based on your needs
      data.kd = 0.15;               // Adapt this based on your needs
      xQueueSend(signalQueue, &data, portMAX_DELAY);
    }
    if (strcmp(effectorIdentifier, "6") == 0)
    {
      data.p_des_position = signal; // Assuming 'position' is available in this scope
      data.id = 0x406;              // Adapt this based on your needs
      data.kp = 20;                 // Adapt this based on your needs
      data.kd = 0.15;               // Adapt this based on your needs
      xQueueSend(signalQueue, &data, portMAX_DELAY);
    }
    if (strcmp(effectorIdentifier, "7") == 0)
    {
      data.p_des_position = signal; // Assuming 'position' is available in this scope
      data.id = 0x407;              // Adapt this based on your needs
      data.kp = 20;                 // Adapt this based on your needs
      data.kd = 0.15;               // Adapt this based on your needs
      xQueueSend(signalQueue, &data, portMAX_DELAY);
    }
  }

  void onEffectorDeregistered(AbstractEffector *effector)
  {
  }

  void onCurvedCustomEventMovementChanged(AbstractEffector *effector, float newMovement)
  {
  }

  void onOnOffCustomEventOnOffChanged(AbstractEffector *effector, bool on)
  {
  }

  void onTriggerCustomEventTriggered(AbstractEffector *effector)
  {
  }

  void onColorCustomEventColorChanged(AbstractEffector *effector, byte newRed, byte newGreen, byte newBlue)
  {
  }

  bool isStepperAutoHomeComplete(AbstractEffector *effector)
  {
    return false;
  }
} // namespace Callbacks
