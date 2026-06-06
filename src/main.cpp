#include <Arduino.h>

#include <RobotMovements.h>
#include <Explorer.h>
#include <Navigator.h>
#include <WifiConn.h>
#include <Connection.h>

#include "Common.h"

void leftTick();
void rightTick();
void initRobot();

RobotMovements rb;
Navigator nav;
Explorer explorer;
WifiConn wifi;
Connection conn;
ServoMotor servo; 
LaserSensor ls;
Ultrasonic ultrasonic;

QueueHandle_t messToClient;

void TaskWeb(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    delay(500);

    messToClient = xQueueCreate(10, sizeof(Message));

    xTaskCreatePinnedToCore(
        TaskWeb,   // Funzione da eseguire
        "TaskWeb", // Nome del task
        10000,     // Stack size
        NULL,      // Parametri
        1,         // Priorità
        NULL,      // Task handle
        0          // CORE 0
    );

    delay(300);

    initRobot();
}

void loop()
{
    explorer.update();
    rb.update();

    yield();
}

void TaskWeb(void *pvParameters)
{
    delay(100);

    wifi.init();
    conn.init(nav, explorer, rb, messToClient);

    Message msg;

    for (;;)
    {
        conn.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void leftTick()
{
    rb.getLeftEnc().increment();
}

void rightTick()
{
    rb.getRightEnc().increment();
}

void initRobot()
{

    Message msg;
    msg.type = INFO;

    explorer.init(&nav, &rb, &servo, &ls, &ultrasonic, messToClient);
    rb.init(&nav, messToClient);
    bool leftEncOk = rb.getLeftEnc().init(leftTick);
    bool rightEncOk = rb.getRightEnc().init(rightTick);

    servo.init();
    bool lsOk = ls.init();
    bool uOk = ultrasonic.init();

    if (!leftEncOk)
    {
        strncpy(msg.mess, "Left enc problem", sizeof(msg.mess) - 1);
        msg.mess[sizeof(msg.mess) - 1] = '\0';
        xQueueSend(messToClient, &msg, 0);
    }

    if (!rightEncOk)
    {
        strncpy(msg.mess, "Right enc problem", sizeof(msg.mess) - 1);
        msg.mess[sizeof(msg.mess) - 1] = '\0';
        xQueueSend(messToClient, &msg, 0);
    }

    if (!lsOk)
    {
        strncpy(msg.mess, "Laser problem", sizeof(msg.mess) - 1);
        msg.mess[sizeof(msg.mess) - 1] = '\0';
        xQueueSend(messToClient, &msg, 0);
    }

    if (!uOk)
    {
        strncpy(msg.mess, "Ultrasonic problem", sizeof(msg.mess) - 1);
        msg.mess[sizeof(msg.mess) - 1] = '\0';
        xQueueSend(messToClient, &msg, 0);
    }
    
}