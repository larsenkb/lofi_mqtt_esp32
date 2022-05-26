//
// lofi_mqtt_esp32.ino:
//	Receive and process lofi security packets.
//	on ESP32
//

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <PubSubClient.h>
#include <SPI.h>
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
int value = 0;

const bool longStr = false;
const bool printPayload = false;
const bool printSeq = false;
const bool en_shockburst = true;
const speed_t speed = speed_250K;
const int rf_chan = 84;
//int maxNodeRcvd = 0;
const bool verbose = false;
volatile uint8_t   gNrfStatus;
//volatile unsigned long currentMillis, lastMillis;
//bool rv;
int mqtt_msg;
int mqtt_pub_err;


QueueHandle_t nrf_queue;
SemaphoreHandle_t sema_MQTT_KeepAlive;


char *nodeMap[] = {
	"node/0",
	"node/1",
	"node/2",
	"node/3",
	"node/4",
	"door/GarageN",			// node/5
	"node/6",
	"node/7",
	"door/Hall",		// node/8
	"node/9",
	"node/10",
	"node/11",
	"node/12",
	"node/13",
	"door/Garage",		  // node/14
	"node/15",
	"node/16",
	"door/GarageS",		  // node/17
	"door/Sliding",		  // node/18
	"door/Back",		    // node/19
	"node/20",
	"window/OfficeN",	  // node/21
	"window/LivingE",		// node/22
	"node/23",
	"window/OfficeS",	  // node/24
	"door/GarageS",	    // node/25
	"window/MasterE",	  // node/26
	"node/27",		      // node/27
	"door/Front",       // node/28
	"node/29",
	"node/30",
	"node/31",
	"door/Sliding",     // node/32
	"door/Office",      // node/33",
	"node/34",
	"node/35",
	"node/36",
	"node/37",
	"node/38",
	"node/99"
};
int maxNodes = sizeof(nodeMap)/sizeof(char*);



void IRAM_ATTR nrfIntrHandler(void)
{
  long sndVal = 1;
  xQueueSendToBack( nrf_queue, &sndVal, portMAX_DELAY );
}

void nrf24_init(void)
{
  // Turn OFF receiving mode, in case it is on...
  digitalWrite(nrfCE, LOW);

  // NRF setup
  // enable 8-bit CRC; mask TX_DS and MAX_RT
  nrfRegWrite( NRF_CONFIG, 0x3c );

  if (en_shockburst) {
    // set nbr of retries and delay
    //  nrfRegWrite( NRF_SETUP_RETR, 0x5F );
    nrfRegWrite( NRF_SETUP_RETR, 0x33 );
    nrfRegWrite( NRF_EN_AA, 1 ); // enable auto ack
  } else {
    nrfRegWrite( NRF_SETUP_RETR, 0 );
    nrfRegWrite( NRF_EN_AA, 0 );
  }

  // Disable dynamic payload
  nrfRegWrite( NRF_FEATURE, 0);
  nrfRegWrite( NRF_DYNPD, 0);

  // Reset STATUS
  nrfRegWrite( NRF_STATUS, 0x70 );

  nrfRegWrite( NRF_EN_RXADDR, 1 ); //3);
  nrfRegWrite( NRF_RX_PW_P0, PAYLOAD_LEN );

  nrfRegWrite( NRF_RX_PW_P1, 0 ); //PAYLOAD_LEN );
  nrfRegWrite( NRF_RX_PW_P2, 0 );
  nrfRegWrite( NRF_RX_PW_P3, 0 );
  nrfRegWrite( NRF_RX_PW_P4, 0 );
  nrfRegWrite( NRF_RX_PW_P5, 0 );

  // Set up channel
  nrfRegWrite( NRF_RF_CH, rf_chan );

  int speedVal = 0x0e;
  switch (speed) {
  case speed_1M:
    speedVal = 0x06;
    break;
  case speed_250K:
    speedVal = 0x26;
    break;
  }
  nrfRegWrite( NRF_RF_SETUP, speedVal | 0);

  nrfFlushTx();
  nrfFlushRx();

  pinMode(NRFIRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(NRFIRQ), nrfIntrHandler, FALLING);
  
  // Power up radio and delay 5ms
  nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x02 );
  delay(5);

  // Enable PRIME RX (PRX)
  nrfRegWrite( NRF_CONFIG, nrfRegRead( NRF_CONFIG ) | 0x01 );

//  if (verbose)
//    nrfPrintDetails();

  // Start receiving lofi packets...
  digitalWrite(nrfCE, HIGH);
  
}


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
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();


  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, NRFIRQ); // sck, miso, mosi, ss (ss can be any GPIO)
  pinMode(nrfCSN, OUTPUT);
  digitalWrite(nrfCSN, HIGH);
  pinMode(nrfCE, OUTPUT);
  digitalWrite(nrfCE, LOW);

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


/*
    Important to not set vTaskDelay to less then 10. Errors begin to develop with the MQTT and network connection.
    makes the initial wifi/mqtt connection and works to keeps those connections open.
*/
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

uint8_t spiXfer(uint8_t *buf, int cnt)
{
  if (buf == (uint8_t*)NULL)
    return 0;
    
  digitalWrite(nrfCSN, LOW);
  for (int i = 0; i < cnt; i++) {
    buf[i] = SPI.transfer(buf[i]);
  }
  digitalWrite(nrfCSN, HIGH);
  gNrfStatus = buf[0];
  return buf[0];
}
