#define RXD2 16    //port number on ESP32 (GPIO16)
#define TXD2 17   //port number on ESP32  (GPIO17)
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>   // Universal Telegram Bot Library written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>

int depth=0;
String text= "h";  


// Initialize Telegram BOT
#define BOTtoken "2147260329:AAHimsPU6TprMq3QE68Sm5K-culxnLYv55o"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "1299672096"
const char* ssid = "Hatem Sultan_EXT";   //SSID 
const char* password = "sultan123"; //password
//----------------------------------------------------------------
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const int ledPin = 2;
bool ledState = LOW;

// Handle what happens when you receive new messages
void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "Use the following commands to control your outputs.\n\n";
      welcome += "/led_on to turn GPIO ON \n";
      welcome += "/led_off to turn GPIO OFF \n";
      welcome += "/state to request current GPIO state \n";
      bot.sendMessage(chat_id, welcome, "");
    }

    if (text == "/led_on") {
      bot.sendMessage(chat_id, "LED state set to ON", "");
      ledState = HIGH;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/led_off") {
      bot.sendMessage(chat_id, "LED state set to OFF", "");
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/state") {
      if (digitalRead(ledPin)){
        bot.sendMessage(chat_id, "LED is ON", "");
      }
      else{
        bot.sendMessage(chat_id, "LED is OFF", "");
      }
    }
  }
}
void setup()   // put your setup code here, to run once:
{
  Serial.begin(9600);      //Baud rate for UART2
  Serial2.begin(9600,SERIAL_8N1,RXD2,TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
   pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
//ESP32 connects to your wifi -----------------------------------
  WiFi.mode(WIFI_STA); //Connect to your wifi
  WiFi.begin(ssid, password);
  #ifdef ESP32
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  #endif
  Serial.println("Connecting to ");
  Serial.print(ssid);

  //Wait for WiFi to connect, scan for Wi-Fi networks, and connect to the strongest of the networks 
  while(WiFi.waitForConnectResult() != WL_CONNECTED)
  {      
      Serial.print(".");
  }    
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
}
//----------------------------------------------------------------
void loop() 
{  
    while(Serial2.available()) //while we're receiving data through the uart from the STM32 board microcontroller
  { 
    depth=Serial2.parseInt(); //converts the received depth into float
    if(depth == 1)
    {
      Serial.print("food\n");   //print the final result of the water depth
      String welcome = "PLEASE REFILL FOOD PLEASEE\n\n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    else
    {
      Serial.print("water\n");   //print the final result of the water depth
      String welcome = "PLEASE REFILL WATER PLEASEE\n\n"; 
      bot.sendMessage(CHAT_ID, welcome, "");
    }
  }
//
//    if (millis() > lastTimeBotRan + botRequestDelay)  {
//    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
//
//    while(numNewMessages) {
//      Serial.println("got response");
//      handleNewMessages(numNewMessages);
//      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
//    }
//    lastTimeBotRan = millis();
  //}

}
