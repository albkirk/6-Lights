// Function to insert customized MQTT Configuration actions

void  mqtt_custom(String reqtopic, StaticJsonDocument<200> data) {
  String reqcomm = String((const char*)data["param"]);            // Should be using the "command" instead of "param"
  //String reqvalue = String((const char*)data["value"]);

  if ( reqcomm == "Info" && bool(data["value"])) {
    //status_report();
    if (BattPowered) mqtt_publish(mqtt_pathtele(), "BattLevel", String(getVoltage()));
    mqtt_publish(mqtt_pathtele(), "Switch", String(SWITCH));
    mqtt_publish(mqtt_pathtele(), "Timer", String(TIMER));
    mqtt_publish(mqtt_pathtele(), "NTP_Sync", String(NTP_Sync));
    mqtt_publish(mqtt_pathtele(), "DateTime", String(curDateTime()));
  }
  if ( reqcomm == "Config" && bool(data["value"])) {
    mqtt_publish(mqtt_pathtele(), "OTA", String(config.OTA));
    mqtt_publish(mqtt_pathtele(), "TELNET", String(config.TELNET));
    mqtt_publish(mqtt_pathtele(), "WEB", String(config.WEB));
    mqtt_publish(mqtt_pathtele(), "Interval", String(config.SLEEPTime));   
  }
  if ( reqcomm == "Switch" && SWITCH_Last == bool(data["value"])) mqtt_publish(mqtt_pathtele(), "Switch", String(SWITCH));

}
