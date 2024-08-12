/*-------------------------------------------------------------------------------------------------
*	Project name: Gesture controlled car
*	File name: esp_32_client.c
*	Author: Shehzeen Malik
*	Date: 26-11-2023
*	Description: This is the client side of the wifi connection to get data from ESP-01 based on
*              Wi-Fi STA Example from espressif.com and randomnerdtutorials (esp32-pwm-arduino-ide)
-------------------------------------------------------------------------------------------------*/
#include <WiFi.h>

//PWM global values
const int gpio_pin1 = 21;  // 21 corresponds to GPIO21
const int gpio_pin2 = 2;

// PWM properties
const int freq = 5000;
const int pwm_channel1 = 1;
const int pwm_channel2 = 0;
const int pwm_resolution = 8; //no of bits for duty cycle value

//wifi global values
const char* ssid     = ""; // wifi SSID of your esp wifi
const char* password = ""; // wifi password of your esp wifi

const char*  host_ip = ""; // IPv4 you assigned to the server
const int wifi_port = ; // Change this to connection port created by server
const String channelID   = ""; // Change this to the channel ID created

WiFiClient client;

//function declaration
void pwm_config(void);
void wifi_config(void);
String readResponse(WiFiClient *client);

void setup() {
  Serial.begin(115200);
  
  //pinMode(BUILTIN_LED, OUTPUT);
  
  pwm_config();

  wifi_config();
  
}

void loop() {
  String car_sig;
  //digitalWrite(BUILTIN_LED, WiFi.status() == WL_CONNECTED);

  car_sig = readResponse(&client);
  Serial.println(car_sig);
  delay(2000);

  if (car_sig == "left"){
    // increase the LED brightness
    for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
      // changing the LED brightness with PWM
      ledcWrite(pwm_channel1, dutyCycle);
      ledcWrite(pwm_channel2, 255-dutyCycle);
      delay(15);
  }
  }
   
   else
   {
    // decrease the LED brightness
    for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
      // changing the LED brightness with PWM
      ledcWrite(pwm_channel1, dutyCycle); 
      ledcWrite(pwm_channel2, 255-dutyCycle);  
      delay(15);
    }
   }
  
}

void pwm_config(void){
  // configure LED PWM functionalitites
  ledcSetup(pwm_channel1, freq, pwm_resolution);
  ledcSetup(pwm_channel2, freq, pwm_resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(gpio_pin1, pwm_channel1);
  ledcAttachPin(gpio_pin2, pwm_channel2);
}

void wifi_config(void){
  /*String readRequest = "GET /channels/" + channelID +
                       "Host: " + host_ip + "\r\n" ;*/
  // connecting to esp01 WiFi network
    Serial.println();
    Serial.println("******************************************************");
    Serial.print("Connecting to ");
    Serial.println(ssid);

    //wifi station mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  
  client.connect(host_ip, wifi_port);
  if (!client.connect(host_ip, wifi_port)) {
    return;
  }
  
  Serial.println("Client connected");
  //client.print(readRequest);
  
  if (!client.connect(host_ip, wifi_port)) {
    return;
  }
}

String readResponse(WiFiClient *client){
  unsigned long timeout = millis();
  while(client->available() == 0){
    if(millis() - timeout > 5000){
      client->stop();
      return ">>> Client Timeout !";
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while(client->available()) {
    String line = client->readStringUntil('\r');
    return line;
  }

 return "\nClosing connection\n\n";
}