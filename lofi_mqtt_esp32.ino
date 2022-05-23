//
// lofi_mqtt_esp32.ino:
//	Receive and process lofi security packets.
//	on ESP32
//

#if defined(ESP32)
#include <WiFi.h>
#include <ArduinoOTA.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <PubSubClient.h>
//#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <stdarg.h>

#define EN_OLED   0
#if EN_OLED
#include <U8x8lib.h>
#endif

#define DEBUG   1
#if DEBUG
#define dbug_printf(fmt, ...) printf(fmt, __VA_ARGS__)
#define dbug_puts(str) puts(str)
#define dbug_putchar(c) putchar(c)
#else
#define dbug_printf(fmt, ...) do {} while (0)
#define dbug_puts(str)
#define dbug_putchar(c)
#endif


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#if defined(ESP32)
#define NRFIRQ			  16
#define nrfCSN        5
#define nrfCE         17

#define MOSI_PIN      23
#define MISO_PIN      19
#define SCLK_PIN      18
#elif defined(ESP8266)
#define NRFIRQ        5
#define nrfCSN        15
#define nrfCE         4

#define MOSI_PIN      13
#define MISO_PIN      12
#define SCLK_PIN      14
#endif

typedef struct {
  uint8_t   nodeId;
  uint8_t   trig        :1;
  uint8_t   closed      :1;
  uint8_t   seq         :2;
  uint8_t   sensorId    :4;
  uint8_t   hi          :4;
  uint8_t   rsvd        :4;
  uint8_t   mid;
  uint8_t   low;
} sensor_t;

#define PAYLOAD_LEN		(sizeof(sensor_t))


#define NRF_CONFIG			  0x00
#define NRF_EN_AA			    0x01
#define NRF_EN_RXADDR		  0x02
#define NRF_SETUP_AW		  0x03
#define NRF_SETUP_RETR		0x04
#define NRF_RF_CH			    0x05
#define NRF_RF_SETUP		  0x06
#define NRF_STATUS			  0x07
#define NRF_OBSERVE_TX		0x08
#define NRF_CD				    0x09
#define NRF_RX_ADDR_P0		0x0A
#define NRF_RX_ADDR_P1		0x0B
#define NRF_RX_ADDR_P2		0x0C
#define NRF_RX_ADDR_P3		0x0D
#define NRF_RX_ADDR_P4		0x0E
#define NRF_RX_ADDR_P5		0x0F
#define NRF_TX_ADDR			  0x10
#define NRF_RX_PW_P0		  0x11
#define NRF_RX_PW_P1		  0x12
#define NRF_RX_PW_P2		  0x13
#define NRF_RX_PW_P3		  0x14
#define NRF_RX_PW_P4		  0x15
#define NRF_RX_PW_P5		  0x16
#define NRF_FIFO_STATUS		0x17
#define NRF_DYNPD			    0x1C
#define NRF_FEATURE			  0x1D

#if (!defined(TRUE))
#define TRUE 1
#endif
	
#if (!defined(FALSE))
#define FALSE !TRUE
#endif


typedef enum {
	SENID_NONE = 0,
	SENID_SW1,
	SENID_SW2,
	SENID_VCC,
	SENID_TEMP,
	SENID_CTR,
	SENID_REV,
  SENID_ATEMP,
  SENID_AHUMD
} senId_t;


typedef enum {
	speed_1M = 0,
	speed_2M = 1,
	speed_250K = 2
} speed_t;

// Please input the SSID and password of WiFi
const char *ssid     = "chewbacca";
const char *password = "Car voice, une oeuvre...";

// MQTT Broker IP address:
const char *mqtt_server = "192.168.1.65";

#define SCL   22
#define SDA   21

#if EN_OLED
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
#endif

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//bool printMqtt = true;
bool longStr = false;
bool printPayload = false;
bool printSeq = false;
bool en_shockburst = true;
speed_t speed = speed_250K;
int rf_chan = 84;
int maxNodeRcvd = 0;
bool verbose = false;
volatile uint8_t   gNrfStatus;
volatile unsigned long currentMillis, lastMillis;
bool rv;


SemaphoreHandle_t sema_nrf;
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



//************  Forward Declarations
void parsePayload( void *pvParameters );
uint8_t spiXfer( uint8_t *buf, int cnt );
uint8_t nrfRegRead( int reg );
uint8_t nrfRegWrite( int reg, int val );
void nrfPrintDetails(void);
int nrfAvailable( uint8_t *pipe_num );
int nrfReadPayload( uint8_t *payload, int len );
uint8_t nrfFlushTx( void );
uint8_t nrfFlushRx( void );
int nrfAddrRead( uint8_t reg, uint8_t *buf, int len );
uint8_t nrfReadRxPayloadLen(void);


void IRAM_ATTR nrfIntrHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR( sema_nrf, &xHigherPriorityTaskWoken );
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
  
  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  if (verbose) dbug_puts("Starting connecting WiFi.");

  sema_nrf = xSemaphoreCreateBinary();
  sema_MQTT_KeepAlive = xSemaphoreCreateBinary();

#if EN_OLED
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.refreshDisplay();    // only required for SSD1606/7  
#endif

  setup_wifi();
  client.setServer(mqtt_server, 1883);


  ArduinoOTA
    .onStart([]() {
      String type;
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
  (void) pvParameters;
  pinMode(LED_BUILTIN, OUTPUT);

  lastMillis = millis();
  
  for (;;) {
    currentMillis = millis();
    if (currentMillis > (lastMillis + (6 * 60 * 1000))) {
      ESP.restart();
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


#define TBUF_LEN        80
#define TOPIC_LEN       40
#define TOPIC_VAL_LEN   40

void parsePayload( void *pvParameters  )
{
  (void) pvParameters;
  int i;
	unsigned short val;
  uint32_t val1;
	uint8_t	sensorId;
	uint8_t nodeId;
	char tbuf[TBUF_LEN+1];
	char topic[TOPIC_LEN];
	char topicVal[TOPIC_VAL_LEN];
	int	tbufIdx;
	int	seq;
  uint8_t payload[PAYLOAD_LEN];
  int pkt_avail = false;
  char u8x8Topic[32];
  char u8x8TopicVal[32];
  sensor_t  *pl = (sensor_t *)payload;
  
  currentMillis = millis();
  lastMillis = currentMillis;
  
  for (;;) {
    
    if (!pkt_avail)
      xSemaphoreTake( sema_nrf, portMAX_DELAY );

    nrfReadPayload( payload, PAYLOAD_LEN );
    nrfRegWrite( NRF_STATUS, 0x70 ); // clear the interrupt

    if (printPayload) {
      dbug_printf("Payload: %02X %02X %02X %02X %02X\n", payload[0], payload[1], payload[2], payload[3], payload[4]);
      goto check;
      //continue;
    }

    // Second byte of payload will never be zero
    if (pl->sensorId == 0 || pl->sensorId > SENID_AHUMD)    // payload[1] == 0)
      goto check;
      //continue;

    nodeId = pl->nodeId;    //payload[0];

	  if (nodeId < 1 || nodeId >= maxNodes) {
		  dbug_printf("Bad nodeId: %d\n", nodeId);
      goto check;
      //continue;
	  }

    sensorId = pl->sensorId;
    seq = pl->seq;
    
    strcpy(topic, "lofi/");
    strcat(topic, nodeMap[nodeId]);
    strcpy(u8x8Topic, nodeMap[nodeId]);
    
	  if (longStr) {
			tbufIdx = snprintf(&tbuf[0], TBUF_LEN, "Id: %2d", nodeId);
	  }

    if (longStr && printSeq) {
      tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Seq: %1d", seq);
    }

    // this might be invalid if this is a switch msg; otherwise it is good
    val = pl->hi;
    val <<= 8;
    val += pl->low;

    val1 = pl->hi;
    val1 = (val1<<8) | (pl->mid & 0xff);
    val1 = (val1<<8) | (pl->low & 0xff);

		switch (sensorId) {
		case SENID_REV:
      strcat(topic, "/rev");
      strcat(u8x8Topic, "/rev");
      sprintf(topicVal, "%d.%d", val1/256, val1&0xff);
      sprintf(u8x8TopicVal, "%d.%d", val1/256, val1&0xff);
			if (longStr) {
			 tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Rev: %d.%d", val1/256, val1&0xff);
			}
			break;
		case SENID_CTR:
      strcat(topic, "/ctr");
      strcat(u8x8Topic, "/ctr");
      sprintf(topicVal, "%d", val);
      sprintf(u8x8TopicVal, "%d", val);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Ctr: %4d", val);
			}
			break;
		case SENID_SW1:
      strcat(topic, "/sw1");
      strcat(u8x8Topic, "/sw1");
      sprintf(topicVal, "{\"state\":\"%s\",\"trig\":\"%s\"}", (pl->closed) ? "OPEN" : "SHUT", (pl->trig) ? "PC" : "WD");
      sprintf(u8x8TopicVal, "%s %s", (pl->closed) ? "OPEN" : "SHUT", (pl->trig) ? "PC" : "WD");
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW1: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
			break;
		case SENID_SW2: // deprecated
      strcat(topic, "/sw1");
      sprintf(topicVal, (pl->closed) ? "OPEN" : "SHUT");
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW2: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
			break;
		case SENID_VCC:
      strcat(topic, "/vcc");
      strcat(u8x8Topic, "/vcc");
      sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
      sprintf(u8x8TopicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
			}
			break;
		case SENID_TEMP:
      strcat(topic, "/temp");
      strcat(u8x8Topic, "/temp");
      sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
      sprintf(u8x8TopicVal, "%4.2f", 1.0 * (float)val - 260.0);
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Temp: %4.2f",1.0 * (float)val - 260.0);
			}
			break;
    case SENID_ATEMP:
      float ftemp;
      strcat(topic, "/atemp");
      strcat(u8x8Topic, "/atemp");
      ftemp = (((float)(val1 * 200))/0x100000) - 50.0;
      ftemp = (ftemp * 1.8) + 32.0;
      if (ftemp < -30.0) ftemp = -30.0;
      if (ftemp > 140.0) ftemp = 140.0;
      ftemp += 0.05;
      sprintf(topicVal, "%4.1f", ftemp);
      sprintf(u8x8TopicVal, "%4.1f", ftemp);
      if (longStr) {
        tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  ATemp: %4.1f", ftemp);
      }
      break;
    case SENID_AHUMD:
      float fhumd;
      strcat(topic, "/ahumd");
      strcat(u8x8Topic, "/ahumd");
      fhumd = ((float)(val1 * 100))/0x100000;
      fhumd += 0.05;
      if (fhumd < 0.0) fhumd = 0.0;
      if (fhumd > 100.0) fhumd = 100.0;
      sprintf(topicVal, "%4.1f", fhumd);
      sprintf(u8x8TopicVal, "%4.1f", fhumd);
      if (longStr) {
        tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  ATemp: %4.1f", fhumd);
      }
      break;
		default:
			dbug_printf("Bad SensorId: %d\n", sensorId);
			goto check;
      //continue;
			break;
		}

    rv = client.publish(topic, topicVal);
    if (!rv) {
      dbug_puts("Error publishing...");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    if (longStr) {
      dbug_printf("%s", tbuf);
    } else {
      dbug_printf("Id: %2d ", nodeId);
      dbug_printf("Seq: %d ", seq);
      dbug_printf("%s %s", topic, topicVal);
    }
    
    if (sensorId == SENID_SW1)
      dbug_printf(" %s\n", (payload[1] & 0x01) ? "PC" : "");
    else
      dbug_puts("");

#if EN_OLED
    u8x8.clear();
    if (u8x8Topic[0] == 'w') {
      i = 0;
      char ch = 'q';
      do {
        ch = u8x8Topic[6+i];
        u8x8Topic[3+i] = ch;
        i++;
      } while (ch);
    }
    u8x8.drawString(0,0,u8x8Topic);
    u8x8.drawString(0,2,u8x8TopicVal);
    u8x8.refreshDisplay();    // only required for SSD1606/7  
#endif

    lastMillis = millis();
    
check:
    pkt_avail = ((nrfRegRead(NRF_FIFO_STATUS) & 1) == 0);
    
//    printf( "parsePayload MEMORY WATERMARK %d\n", uxTaskGetStackHighWaterMark(NULL) );

  } //endof: for(;;)
  
  // should never get here
  vTaskDelete ( NULL );
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


int nrfAddrRead( uint8_t reg, uint8_t *buf, int len )
{
	if (buf && len > 1) {
		buf[0] = reg & 0x1f;
		spiXfer(buf, len+1);
		return buf[1];
	}
	return -1;
}


uint8_t nrfFlushRx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe2;
	return spiXfer(spiBuf, 1);
}

uint8_t nrfFlushTx( void )
{
	uint8_t spiBuf[1];

	spiBuf[0] = 0xe1;
	return spiXfer(spiBuf, 1);
}

uint8_t nrfRegWrite( int reg, int val)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x20 | (reg & 0x1f);
	spiBuf[1] = val;
	return spiXfer(spiBuf, 2);
}

uint8_t nrfRegRead( int reg )
{
	uint8_t spiBuf[2];

	spiBuf[0] = reg & 0x1f;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

uint8_t nrfReadRxPayloadLen(void)
{
	uint8_t spiBuf[2];

	spiBuf[0] = 0x60;
	spiBuf[1] = 0;
	spiXfer(spiBuf, 2);
	return spiBuf[1];
}

int nrfAvailable( uint8_t *pipe_num )
{
	uint8_t status;

	status = nrfRegRead( NRF_STATUS );
	if (status & 0x40 ) {
		if ( pipe_num ) {
			*pipe_num = ((status>>1) & 0x7);
		}
		return 1;
	}
	return 0;
}

int nrfReadPayload( uint8_t *payload, int len )
{
	uint8_t spiBuf[33];
	int i;

	if (len > 32)
		return -1;
	if (len < 1)
		return -1;

	spiBuf[0] = 0x61;
	for (i = 1; i < len+1; i++)
		spiBuf[i] = 0;
	spiXfer(spiBuf, len+1);
	if (payload)
		for (i = 1; i < len+1; i++)
			payload[i-1] = spiBuf[i];
	
//	nrfRegWrite( NRF_STATUS, 0x70 ); // clear the interrupt

	return 0;
}


void nrfPrintDetails(void)
{
	uint8_t		buf[6];

	dbug_puts("================ SPI Configuration ================" );
	dbug_printf("CSN Pin  \t = Custom GPIO%d\n", nrfCSN  );
	dbug_printf("CE Pin  \t = Custom GPIO%d\n", nrfCE );
	dbug_puts("Clock Speed\t = " );
	dbug_puts("1 Mhz");
	dbug_puts("================ NRF Configuration ================");
 

	dbug_printf("STATUS: %02X\n", nrfRegRead( NRF_STATUS ));
	nrfAddrRead( NRF_RX_ADDR_P0, buf, 5 );
	dbug_printf("RX_ADDR_P0: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P0: %02X\n", nrfRegRead( NRF_RX_ADDR_P0 ));
	nrfAddrRead( NRF_RX_ADDR_P1, buf, 5 );
	dbug_printf("RX_ADDR_P1: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//	printf("RX_ADDR_P1: %02X\n", nrfRegRead( NRF_RX_ADDR_P1 ));
	dbug_printf("RX_ADDR_P2: %02X\n", nrfRegRead( NRF_RX_ADDR_P2 ));
	dbug_printf("RX_ADDR_P3: %02X\n", nrfRegRead( NRF_RX_ADDR_P3 ));
	dbug_printf("RX_ADDR_P4: %02X\n", nrfRegRead( NRF_RX_ADDR_P4 ));
	dbug_printf("RX_ADDR_P5: %02X\n", nrfRegRead( NRF_RX_ADDR_P5 ));
//	printf("TX_ADDR: %02X\n", nrfRegRead( NRF_TX_ADDR ));
	nrfAddrRead( NRF_TX_ADDR, buf, 5 );
	dbug_printf("TX_ADDR: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);

//  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
	dbug_printf("EN_AA: %02X\n", nrfRegRead( NRF_EN_AA ));
	dbug_printf("EN_RXADDR: %02X\n", nrfRegRead( NRF_EN_RXADDR ));
	dbug_printf("RF_CH: %02X\n", nrfRegRead( NRF_RF_CH ));
	dbug_printf("RF_SETUP: %02X\n", nrfRegRead( NRF_RF_SETUP ));
	dbug_printf("RX_PW_P0: %02X\n", nrfRegRead( NRF_RX_PW_P0 ));
	dbug_printf("RX_PW_P1: %02X\n", nrfRegRead( NRF_RX_PW_P1 ));
	dbug_printf("RX_PW_P2: %02X\n", nrfRegRead( NRF_RX_PW_P2 ));
	dbug_printf("RX_PW_P3: %02X\n", nrfRegRead( NRF_RX_PW_P3 ));
	dbug_printf("RX_PW_P4: %02X\n", nrfRegRead( NRF_RX_PW_P4 ));
	dbug_printf("RX_PW_P5: %02X\n", nrfRegRead( NRF_RX_PW_P5 ));
	dbug_printf("CONFIG: %02X\n", nrfRegRead( NRF_CONFIG ));
	dbug_printf("CD: %02X\n", nrfRegRead( NRF_CD ));
	dbug_printf("SETUP_AW: %02X\n", nrfRegRead( NRF_SETUP_AW ));
	dbug_printf("SETUP_RETR: %02X\n", nrfRegRead( NRF_SETUP_RETR ));
	dbug_printf("DYNPD: %02X\n", nrfRegRead( NRF_DYNPD ));
	dbug_printf("FEATURE: %02X\n", nrfRegRead( NRF_FEATURE ));

	if (speed == speed_1M)
		dbug_printf("Data Rate\t = %s\n", "1Mbps" );
	else if (speed == speed_250K)
		dbug_printf("Data Rate\t = %s\n", "250Kbps" );
	else
		dbug_printf("Data Rate\t = %s\n", "2Mbps" );

	dbug_printf("Model\t\t = %s\n", "nRF24L01+"  );
	dbug_printf("CRC Length\t = %s\n", "8 bits");
	dbug_printf("PA Power\t = %s\n", "PA_MAX" );

}
