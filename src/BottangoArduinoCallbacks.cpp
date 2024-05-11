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

unsigned long lastTrueTime = 0; // stores the last time allPositionsWithinRange was true

const int interval = 0;       // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10; // Receive Queue size

const int nOfMotors = 2; // Number of motors

float currentMotorPositions[nOfMotors]; // Array to store positions of 7 motors
bool receivedMotorPositions[nOfMotors]; // Array to store if the position of a motor has been received
float targetMotorPositions[nOfMotors];

const int AUTO_HOME_SPEED = 2000;

bool autoHoming = false;
bool ahSent = false;

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

  float calculateEncoderPosition(float encoder, bool singleTurn = true)
  { // singleTurn == true returns single turn position, false returns multi-turn position
    if (singleTurn)
    {
      float position = fmod((encoder / 65536.0 * 360.0), 360.0);
      if (position < 0)
      {
        position += 360.0;
      }
      return position;
    }
    else
    {
      return encoder / 65536.0 * 360.0;
    }
  }

  float calculateAngleDistance(float angle1, float angle2)
  {
    return abs(angle1 - angle2);
  }

  void requestMotorPositions()
  {
    for (int motorIndex = 0; motorIndex < nOfMotors; motorIndex++)
    {
      int motorId = 0x140 + motorIndex + 1;
      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = motorId;   // CAN ID
      tx_frame.FIR.B.DLC = 8;     // Data length
      tx_frame.data.u8[0] = 0x60; // Command byte to request position
      for (int i = 1; i < 8; i++)
      {
        tx_frame.data.u8[i] = 0x00; // NULL rest of data
      }

      ESP32Can.CANWriteFrame(&tx_frame);

      CAN_frame_t rx_frame;
      if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE)
      {
        if (rx_frame.MsgID >= 0x240 && rx_frame.MsgID <= 0x240 + nOfMotors)
        { // Check for correct response IDs
          int32_t encoder = rx_frame.data.u8[4] | (rx_frame.data.u8[5] << 8) |
                            (rx_frame.data.u8[6] << 16) | (rx_frame.data.u8[7] << 24);
          int motorMessageIndex = rx_frame.MsgID - 0x240; // Calculate motor index based on ID
          if (motorMessageIndex >= 1 && motorMessageIndex <= nOfMotors)
          {                                                                                          // Ensure the index is within bounds
            currentMotorPositions[motorMessageIndex - 1] = calculateEncoderPosition(encoder, false); // Store position in the array
          }
        }
      }
    }
  }

  void sendCustomMotionControlCommand(MotionControlData data)
  {
    if (!autoHoming)
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
    }
    else
    {
      int currentId = data.id - 0x400;
      if (currentId >= 1 && currentId <= nOfMotors)
      {
        receivedMotorPositions[currentId - 1] = true;
        targetMotorPositions[currentId - 1] = map(data.p_des_position, 1, 3700, 0, 30);
        // Serial.print("m1 " + String(currentId) + " target: " + String(targetMotorPositions[currentId - 1]) + " current: " + String(currentMotorPositions[currentId - 1]) + " distance: " + String(calculateAngleDistance(currentMotorPositions[currentId - 1], targetMotorPositions[currentId - 1])));
      }

      bool allReceived = true;

      for (int i = 0; i < nOfMotors; i++)
      {
        if (!receivedMotorPositions[i])
        {
          allReceived = false;
          break;
        }
      }

      requestMotorPositions();

      bool allPositionsWithinRange = true;

      for (int i = 0; i < nOfMotors; i++)
      {
        if (calculateAngleDistance(currentMotorPositions[i], targetMotorPositions[i]) > 1)
        {
          Serial.println("M" + String(i + 1) + " dist: " + String(calculateAngleDistance(currentMotorPositions[i], targetMotorPositions[i])));
          allPositionsWithinRange = false;
          break;
        }
      }

      if (allPositionsWithinRange)
      {

        delay(100);
        unsigned long currentTime = millis();
        if (currentTime - lastTrueTime > 500)
        {
          for (int i = 0; i < nOfMotors; i++)
          {
            Stop_Motor(i + 1);
          }
          autoHoming = false;
          ahSent = false;
          allReceived = false;
          for (int i = 0; i < nOfMotors; i++)
          {
            receivedMotorPositions[i] = false;
          }

          Serial.println("Auto homing complete");
          strip.setPixelColor(0, strip.Color(0, 20, 0)); // Green color
          strip.show();
        }
        else
        {
          lastTrueTime = currentTime;
        }
      }
      else
      {

        if (allReceived && !ahSent)
        {
          ESP32Can.CANInit();
          for (int i = 0; i < nOfMotors; i++)
          {
            Stop_Motor(i + 1);
          }
          delay(100);
          ahSent = true;
          for (int i = 0; i < nOfMotors; i++)
          {
            int destinationAngle = targetMotorPositions[i] * 100;
            Serial.println("Sending motor " + String(i + 1) + " to " + String(destinationAngle));
            sendMotorCommand(destinationAngle, AUTO_HOME_SPEED, 0x140 + i + 1);
            delay(30);
          }
        }

        strip.setPixelColor(0, strip.Color(20, 15, 0)); // Yellow color
        strip.show();
      }
    }

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
    autoHoming = true;
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
    // delay(10);
    // Motor1_Stop();
    // Motor2_Stop();
    // delay(10);
    // sendMotorCommand(1, 50, 0x141);
    // sendMotorCommand(1, 50, 0x142);
    // delay(7000);
    // Motor1_Stop();
    // Motor2_Stop();

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
