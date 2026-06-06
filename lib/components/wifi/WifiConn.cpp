#include "WifiConn.h"
#include <WiFi.h>

const char *ssid = "PicoW";
const char *password = "slamtest";

void WifiConn::init()
{
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_AP);

    IPAddress local_IP(192, 168, 1, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 255, 0);

    if (WiFi.softAPConfig(local_IP, gateway, subnet))
    {
        if (WiFi.softAP(ssid, password, 1, _hidden, _maxConn))
        {
            WiFi.setTxPower(WIFI_POWER_15dBm); 
            
            Serial.print("AP created. IP: ");
            Serial.println(WiFi.softAPIP());
        }
        else
        {
            Serial.println("Error Access Point");
        }
    }
    else
    {
        Serial.println("Error setting IP");
    }
}