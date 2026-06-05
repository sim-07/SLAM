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
    // while (!Serial);

    messToClient = xQueueCreate(10, sizeof(Message));

    initRobot();

    xTaskCreatePinnedToCore(
        TaskWeb,   // Funzione da eseguire
        "TaskWeb", // Nome del task
        10000,     // Stack size
        NULL,      // Parametri
        1,         // Priorità
        NULL,      // Task handle
        0          // CORE 0
    );
}

void loop()
{
    explorer.update();
    rb.update();
    
    yield();
}

void TaskWeb(void *pvParameters) {
    delay(500);
    
    wifi.init();
    conn.init(nav, explorer, rb, messToClient);

    Message msg;

    for (;;) {
        conn.update();

        // if (xQueueReceive(messToClient, &msg, 0) == pdTRUE) {
        //     conn.sendMessage(msg);
        // }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void setup1()
// {
//     delay(500);

//     wifi.init();
//     conn.init(nav, explorer);
// }

// void loop1()
// {
//     conn.update();
// }

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

    explorer.init(&nav, &rb, &servo, &ls, &ultrasonic, messToClient);
    rb.init(&nav, messToClient);
    rb.getLeftEnc().init(leftTick);
    rb.getRightEnc().init(rightTick);

    servo.init();

    ls.init();

    ultrasonic.init();
}
