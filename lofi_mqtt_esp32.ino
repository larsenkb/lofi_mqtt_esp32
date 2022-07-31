//
// lofi_mqtt_esp32.ino:
//	Receive and process lofi security packets on ESP32.
//  Upload (reprogram ESP32) over the air (OTA).
//

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>
#include <stdio.h>
#include <stdarg.h>

#include "lofi.h"


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Please input the SSID and password of WiFi
const char *ssid     = "chewbacca";
const char *password = "Car voice, une oeuvre...";

// MQTT Broker IP address:
const char *mqtt_server = "192.168.1.65";


WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];

const bool verbose = false;
int mqtt_msg;
int mqtt_pub_err;


QueueHandle_t nrf_queue;
SemaphoreHandle_t sema_MQTT_KeepAlive;



//
// setup
//
void setup(void)
{

  Serial.begin(115200);

  mqtt_msg = 0;
  mqtt_pub_err = 0;

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  if (verbose) dbug_puts("Starting connecting WiFi.");

  nrf_queue = xQueueCreate( 8, sizeof( long ) );
  sema_MQTT_KeepAlive = xSemaphoreCreateBinary();

  setup_wifi();
  client.setServer(mqtt_server, 1883);

ArduinoOTA
    .onStart([]() {
      String type;
      detachInterrupt(digitalPinToInterrupt(NRFIRQ));
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      Serial.print("Progress: ");
      Serial.print((progress / (total / 100)));
      Serial.print("%\r");
    })
    .onError([](ota_error_t error) {
      //Serial.printf("Error[%u]: ", error);
      Serial.print("Error: ");
      Serial.println(error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();


//  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, NRFIRQ); // sck, miso, mosi, ss (ss can be any GPIO)

  // stack size is specified in bytes for ESP32 FreeRTOS, not words...
  // lower number - lower priority
  xTaskCreatePinnedToCore( parsePayload,  "parsePayload",  4000, NULL, 5, NULL, 1 ); // assign all to core 1, WiFi in use.
  xTaskCreatePinnedToCore( MQTTkeepalive, "MQTTkeepalive", 4000, NULL, 3, NULL, 1 ); // this task makes a WiFi and MQTT connection.
  xTaskCreatePinnedToCore( blink, "blink", 2000, NULL, 2, NULL, 1 ); // this task makes a WiFi and MQTT connection.

  nrf24_init();

} // endof: setup


void loop() 
{
    ArduinoOTA.handle();
}


void blink( void *pvParameters )
{
  volatile unsigned long currentMillis, startMillis;
  int local_mqtt_msg = mqtt_msg;
  int local_mqtt_pub_err = mqtt_pub_err;
  
  (void) pvParameters;
  pinMode(LED_BUILTIN, OUTPUT);

  startMillis = millis();
  
  for (;;) {
    currentMillis = millis();
//    printf("ho "); fflush(stdout);
    if (currentMillis > (startMillis + (6 * 60 * 1000))) {
      startMillis = currentMillis;
      if (local_mqtt_msg == mqtt_msg) {
        Serial.print("restarting rcvd no pkts...");
        Serial.println(mqtt_msg);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP.restart();
      } else {
        local_mqtt_msg = mqtt_msg;
      }
      if (mqtt_pub_err > (local_mqtt_pub_err + 5)) {
        Serial.print("restarting mqtt publish errors...");
        Serial.println(mqtt_pub_err);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP.restart();
      } else {
        local_mqtt_pub_err = mqtt_pub_err;
      }
    }
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(10/ portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(4990/ portTICK_PERIOD_MS);
  }
}


//
// Important to not set vTaskDelay to less then 10. Errors begin to develop with the MQTT and network connection.
// makes the initial wifi/mqtt connection and works to keeps those connections open.
//
void MQTTkeepalive( void *pvParameters )
{
  // setting must be set before a mqtt connection is made
  client.setKeepAlive( 90 ); // setting keep alive to 90 seconds makes for a very reliable connection, must be set before the 1st connection is made.

  for (;;) {
    //check for a is connected and if the WiFi thinks its connected, found checking on both is more realible than just a single check
    if ( (espClient.connected()) && (WiFi.status() == WL_CONNECTED) ) {
      xSemaphoreTake( sema_MQTT_KeepAlive, portMAX_DELAY ); // whiles loop() is running no other mqtt operations should be in process
      client.loop();
      xSemaphoreGive( sema_MQTT_KeepAlive );
    } else {
      if (verbose) dbug_printf( "MQTT keep alive found MQTT status %s WiFi status %s\n", String(espClient.connected()), String(WiFi.status()) );
      if ( !(WiFi.status() == WL_CONNECTED) ) {
        connectToWiFi();
      }
      connectToMQTT();
    }
    vTaskDelay( 250 / portTICK_PERIOD_MS); //task runs approx every 250 mS
  }
  vTaskDelete ( NULL );
}

int mqtt_pub(char *topic, char *topicVal)
{
    return client.publish(topic, topicVal);
}

void connectToMQTT()
{
  if (verbose) dbug_puts( "connect to mqtt" );
  while ( !client.connected() ) {
    client.connect("lofi_mqtt_esp32-1");
    //client.connect( clientID, mqtt_username, mqtt_password );
    if (verbose) dbug_puts( "connecting to MQTT" );
    vTaskDelay( 250 / portTICK_PERIOD_MS);
  }
  if (verbose) dbug_puts("MQTT Connected");
}

void setup_wifi()
{
  IPAddress ip;
  byte mac[6];
  
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // We start by connecting to a WiFi network
  if (verbose) dbug_printf("Connecting to %s\n", ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (verbose) dbug_putchar('.');
  }

  ip = WiFi.localIP();
  if (verbose) dbug_printf("\nWiFi connected: IP: %d.%d.%d.%d\n", ip[0],ip[1],ip[2],ip[3]);

  //dbug_printf("\nWiFi connected: IP address: %s\n", WiFi.localIP().toString());

  WiFi.macAddress(mac);
  if (verbose) dbug_printf( "MAC %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]  );
  WiFi.onEvent( WiFiEvent );
}


void connectToWiFi()
{
  byte mac[6];
  
  if (verbose) dbug_puts( "connect to wifi" );
  while ( WiFi.status() != WL_CONNECTED ) {
    WiFi.disconnect();
    WiFi.begin( ssid, password );
    if (verbose) dbug_puts(" waiting on wifi connection" );
    vTaskDelay( 4000 / portTICK_PERIOD_MS);
  }
  if (verbose) dbug_puts( "Connected to WiFi" );
  WiFi.macAddress(mac);
  if (verbose) dbug_printf( "mac address %02X.%02X.%02X.%02X.%02X.%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]  );
  WiFi.onEvent( WiFiEvent );
}

////
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
  case SYSTEM_EVENT_STA_CONNECTED:
    if (verbose) dbug_puts("Connected to access point");
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    if (verbose) dbug_puts("Disconnected from WiFi access point");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    if (verbose) dbug_puts("WiFi client disconnected");
    break;
  default:
    break;
  }
}
