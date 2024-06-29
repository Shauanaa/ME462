#include <ESP8266WiFi.h>
#include <espnow.h>

#define CHANNEL 1
#define AGENT_ID 1

#define ENA 4
#define IN1 2
#define IN2 14
#define ENB 5
#define IN3 13
#define IN4 12

#define FAN 16

int speed_val_left = 0;
int speed_val_right = 0;
int speed_val_fan = 0;
volatile uint32_t counter = 0;
char buff[20];

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  //Serial.printf("Received from MAC: %2x:%2x:%2x:%2x:%2x:%2x!\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if(len != 14) return;
  memcpy(buff, incomingData, len);
  buff[len] = '\0';
  Serial.printf("%s\n", buff);
  if(incomingData[0] != 'a' + AGENT_ID) return;
  memcpy(buff, incomingData + 1, 4);
  buff[4] = '\0';
  speed_val_left = atoi(buff);
  memcpy(buff, incomingData + 5, 4);
  buff[4] = '\0';
  speed_val_right = atoi(buff);
  memcpy(buff, incomingData + 9, 4);
  buff[4] = '\0';
  speed_val_fan = 255 - atoi(buff);
  motorControl(speed_val_left, speed_val_right);
  analogWrite(FAN, speed_val_fan);
  counter = millis();
  Serial.printf("Cmd recv: %+04d\t %+04d\t %+04d\n", speed_val_left, speed_val_right, speed_val_fan);
}
 
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(FAN, OUTPUT);

  Serial.begin(115200);
  Serial.print("\nStarting WiFi!\nESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_AP);
  char wifi_name[20];
  sprintf(wifi_name, "AGENT_%d", AGENT_ID);
  WiFi.softAP(wifi_name, "Romer1234", CHANNEL, 0);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if(millis()-counter > 500){
    motorControl(0, 0);
    digitalWrite(FAN, HIGH);
    Serial.println("Timeout!");
    counter = millis();
  }
}
