#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

//kanal belirliyoruz alıcı ve verici aynı kanalda olmalı 
#define CHANNEL 1
#define MAX_AGENTS 10

uint8_t agents[MAX_AGENTS][6];
bool agents_present[MAX_AGENTS];
uint8_t buffer[20];

void setup ()
{
  buffer[10] = '\0';
  Serial.begin(115200);
  Serial.print("\nStarting WiFi!\nESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  //esp_now_register_send_cb(OnDataSent);
  EEPROM.begin(MAX_AGENTS*7);
  LoadFromEEPROM();
  ScanForSlave();
  SaveToEEPROM();
}

void OnDataSent(uint8_t *mac_addr, uint8_t status ) {
  Serial.printf("Sent: %d\n", status);
}

void loop(){
  if(Serial.available() > 0){
    //Serial.printf("Received:\n");
    if(Serial.readBytesUntil('\n', buffer, 14) == 13){
      //buffer[9] = '\0';
      //Serial.printf("%s\n", buffer);
      for(int agent_id = 0; agent_id < MAX_AGENTS; agent_id++){
        if(('a'+agent_id == buffer[0]) && (agents_present[agent_id] == true)){
          buffer[13] = '\n';
          //Serial.printf("%s",buffer);
          esp_now_send(agents[agent_id], buffer, 14);
        }
      }
    }
  }
}

void LoadFromEEPROM(){
  Serial.println("--------------------------");
  Serial.println("Loaded from EEPROM:");
  for(int i = 0; i < MAX_AGENTS;i++){
    for(int j = 0; j < 6;j++){
      EEPROM.get(j+i*6, agents[i][j]);
    }
  }
  for(int i = 0; i < MAX_AGENTS;i++){
    EEPROM.get(MAX_AGENTS*6 + i, agents_present[i]);
    if(agents_present[i]){
      Serial.printf("Agent %d MAC: %02x:%02x:%02x:%02x:%02x:%02x!\n", i, agents[i][0], agents[i][1], agents[i][2], agents[i][3], agents[i][4], agents[i][5]);
    }
  }
  Serial.println("--------------------------");
}

void SaveToEEPROM(){
  Serial.println("--------------------------");
  Serial.println("Saved to EEPROM:");
  for(int i = 0; i < MAX_AGENTS;i++){
    for(int j = 0; j < 6;j++){
      EEPROM.put(j+i*6, agents[i][j]);
    }
  }
  for(int i = 0; i < MAX_AGENTS;i++){
    EEPROM.put(MAX_AGENTS*6 + i, agents_present[i]);
    if(agents_present[i]){
      Serial.printf("Agent %d MAC: %02x:%02x:%02x:%02x:%02x:%02x!\n", i, agents[i][0], agents[i][1], agents[i][2], agents[i][3], agents[i][4], agents[i][5]);
    }
  }
  EEPROM.commit();
  Serial.println("--------------------------");
}

void ScanForSlave() 
{
  int16_t scanResults = WiFi.scanNetworks(); // Scan only on one channel
  for (int i = 0; i < scanResults; ++i) {
    // Print SSID and RSSI for each device found
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);
    for(int agent_id = 0; agent_id < MAX_AGENTS; agent_id++){
      String agent_name("AGENT_");
      agent_name.concat(agent_id);
      if (SSID.compareTo(agent_name) == 0){
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int j = 0; j < 6; ++j ) {
            agents[agent_id][j] = (uint8_t) mac[j];
          }
        }
        esp_now_add_peer(agents[agent_id],ESP_NOW_ROLE_SLAVE,1,NULL,0);
        agents_present[agent_id] = true;
        Serial.printf("Agent %d found with MAC: %02x:%02x:%02x:%02x:%02x:%02x!\n", agent_id, agents[agent_id][0], agents[agent_id][1], agents[agent_id][2], agents[agent_id][3], agents[agent_id][4], agents[agent_id][5]);
      }
    }
  }
}