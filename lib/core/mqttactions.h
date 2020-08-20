// Adding function with project's customized MQTT actions
#include <custommqtt.h>

// Handling of received message
void on_message(const char* topic, byte* payload, unsigned int msg_length) {

    telnet_println("New message received from Broker");

    char msg[msg_length + 1];
    strncpy (msg, (char*)payload, msg_length);
    msg[msg_length] = '\0';

    telnet_println("Topic: " + String(topic));
    telnet_println("Payload: " + String((char*)msg));

    // Decode JSON request
    StaticJsonDocument<256> data;
    DeserializationError JSONerror = deserializeJson(data, msg);

    if (JSONerror) {
      telnet_println("JSON deserialization failed!. Error code: " + String(JSONerror.c_str()));
      return;
    }

    // Check request method
    String reqtopic = String(topic);
    String reqcomm = String((const char*)data["param"]);      // Should be using the "command" instead of "param"
    String reqvalue = String((const char*)data["value"]);
    if (data["value"].is<const char*>() == false) reqvalue = String((long)data["value"]);
    telnet_println("Received Data: " + reqcomm + " = " + reqvalue);


    // System Configuration 
    if ( reqcomm == "DeviceName") strcpy(config.DeviceName, (const char*)data["value"]);
    if ( reqcomm == "Location") strcpy(config.Location, (const char*)data["value"]);
    if ( reqcomm == "ClientID") strcpy(config.ClientID, (const char*)data["value"]);
    if ( reqcomm == "DEEPSLEEP") { config.DEEPSLEEP = bool(data["value"]);storage_write(); }
    if ( reqcomm == "SLEEPTime") { config.SLEEPTime = data["value"];storage_write(); }
    if ( reqcomm == "ONTime") { config.ONTime = data["value"];storage_write(); }
    if ( reqcomm == "ExtendONTime") if (bool(data["value"]) == true) Extend_time = 60;
    if ( reqcomm == "LED") config.LED = bool(data["value"]);
    if ( reqcomm == "TELNET") { config.TELNET = bool(data["value"]); storage_write(); telnet_setup(); }
    if ( reqcomm == "OTA") { config.OTA = bool(data["value"]); storage_write(); ESPRestart(); }
    if ( reqcomm == "WEB") { config.WEB = bool(data["value"]); storage_write(); web_setup(); }
    if ( reqcomm == "DHCP") { config.DHCP = bool(data["value"]); storage_write(); wifi_connect(); }
    if ( reqcomm == "STAMode") { config.STAMode = bool(data["value"]); storage_write(); }
    if ( reqcomm == "APMode") { config.APMode = bool(data["value"]); storage_write(); }
    if ( reqcomm == "SSID") strcpy(config.SSID, (const char*)data["value"]);
    if ( reqcomm == "WiFiKey") strcpy(config.WiFiKey, (const char*)data["value"]);
    if ( reqcomm == "NTPServerName") strcpy(config.NTPServerName, (const char*)data["value"]);
    if ( reqcomm == "MQTT_Server") { strcpy(config.MQTT_Server, (const char*)data["value"]); storage_write(); }
    if ( reqcomm == "MQTT_Port") { config.MQTT_Port = data["value"];storage_write(); }
    if ( reqcomm == "MQTT_Secure") { config.MQTT_Secure = bool(data["value"]); storage_write(); }
    if ( reqcomm == "MQTT_User") { strcpy(config.MQTT_User, (const char*)data["value"]); storage_write(); }
    if ( reqcomm == "MQTT_Password") { strcpy(config.MQTT_Password, (const char*)data["value"]); storage_write(); }   
    if ( reqcomm == "Update_Time_Via_NTP_Every") config.Update_Time_Via_NTP_Every = data["value"];
    if ( reqcomm == "TimeZone") config.TimeZone = data["value"];
    if ( reqcomm == "isDayLightSaving") config.isDayLightSaving = bool(data["value"]);
    if ( reqcomm == "Store") if (bool(data["value"]) == true) storage_write();
    if ( reqcomm == "Boot") if (bool(data["value"]) == true) mqtt_restart();
    if ( reqcomm == "Reset") if (bool(data["value"]) == true) storage_reset();
    if ( reqcomm == "Switch_Def") { 
            config.SWITCH_Default = bool(data["value"]);
            storage_write();
            mqtt_publish(mqtt_pathtele(), "Switch", String(SWITCH));
       }
    if ( reqcomm == "Temp_Corr") { 
            config.Temp_Corr = float(data["value"]);
            storage_write();
            mqtt_publish(mqtt_pathtele(), "Temperatura", String(getTemperature()));
       }
    if ( reqcomm == "LDO_Corr") { 
            config.LDO_Corr = float(data["value"]);
            storage_write();
            mqtt_publish(mqtt_pathtele(), "BattLevel", String(getVoltage()));
       }

    // Standard Actuators/Actions 
    if ( reqcomm == "Level") LEVEL = uint(data["value"]);
    if ( reqcomm == "Position") POSITION = int(data["value"]);
    if ( reqcomm == "Switch") SWITCH = bool(data["value"]);
    if ( reqcomm == "Timer") TIMER = ulong(data["value"]);
    if ( reqcomm == "Counter") COUNTER = ulong(data["value"]);
    if ( reqcomm == "Calibrate") { CALIBRATE = float(data["value"]); }


    mqtt_custom(reqtopic, data);

    storage_print();
}


// The callback to handle the MQTT PUBLISH messages received from Broker.
void mqtt_setcallback() {
    MQTTclient.setCallback(on_message);
}


// MQTT commands to run on setup function.
void mqtt_setup() {
    mqtt_connect();
    if (MQTT_state == MQTT_CONNECTED) {
        if (ESPWakeUpReason() == "Deep-Sleep Wake") mqtt_publish(mqtt_pathtele(), "Status", "WakeUp");
        else {
            mqtt_publish(mqtt_pathtele(), "Boot", ESPWakeUpReason());
            mqtt_publish(mqtt_pathtele(), "Brand", BRANDName);
            mqtt_publish(mqtt_pathtele(), "Model", MODELName);
            mqtt_publish(mqtt_pathtele(), "ChipID", ChipID);
            mqtt_publish(mqtt_pathtele(), "SWVer", SWVer);
            status_report();
            mqtt_publish(mqtt_pathtele(), "IP", WiFi.localIP().toString());
            mqtt_publish(mqtt_pathtele(), "RSSI", String(getRSSI()));
        }
        mqtt_dump_data();
    }
    mqtt_setcallback();
}


// MQTT commands to run on loop function.
void mqtt_loop() {
    if (!MQTTclient.loop()) {
        if ( millis() - MQTT_LastTime > (MQTT_Retry * 1000)) {
            MQTT_errors ++;
            Serial.println( "in loop function MQTT ERROR! #: " + String(MQTT_errors) + "  ==> " + MQTT_state_string(MQTTclient.state()) );
            MQTT_LastTime = millis();
            mqtt_connect();
            status_report();
            mqtt_dump_data();
        }
    }
    yield();
}