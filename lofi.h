#ifndef __LOFI_H__
#define __LOFI_H__

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <freertos/queue.h>

#if defined(ESP32)
#define NRFIRQ        16
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

#define LONG_STR      0

#define DEBUG         0

#if DEBUG
#define dbug_printf(fmt, ...) printf(fmt, __VA_ARGS__)
#define dbug_puts(str) puts(str)
#define dbug_putchar(c) putchar(c)
#else
#define dbug_printf(fmt, ...) do {} while (0)
#define dbug_puts(str)
#define dbug_putchar(c)
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

#define PAYLOAD_LEN   (sizeof(sensor_t))


#define NRF_CONFIG        0x00
#define NRF_EN_AA         0x01
#define NRF_EN_RXADDR     0x02
#define NRF_SETUP_AW      0x03
#define NRF_SETUP_RETR    0x04
#define NRF_RF_CH         0x05
#define NRF_RF_SETUP      0x06
#define NRF_STATUS        0x07
#define NRF_OBSERVE_TX    0x08
#define NRF_CD            0x09
#define NRF_RX_ADDR_P0    0x0A
#define NRF_RX_ADDR_P1    0x0B
#define NRF_RX_ADDR_P2    0x0C
#define NRF_RX_ADDR_P3    0x0D
#define NRF_RX_ADDR_P4    0x0E
#define NRF_RX_ADDR_P5    0x0F
#define NRF_TX_ADDR       0x10
#define NRF_RX_PW_P0      0x11
#define NRF_RX_PW_P1      0x12
#define NRF_RX_PW_P2      0x13
#define NRF_RX_PW_P3      0x14
#define NRF_RX_PW_P4      0x15
#define NRF_RX_PW_P5      0x16
#define NRF_FIFO_STATUS   0x17
#define NRF_DYNPD         0x1C
#define NRF_FEATURE       0x1D

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


extern QueueHandle_t nrf_queue;
//extern SemaphoreHandle_t sema_nrf;
extern SemaphoreHandle_t sema_MQTT_KeepAlive;

extern const bool printPayload;
extern char *nodeMap[];
extern int maxNodes;
extern const bool longStr;
extern const bool printSeq;
extern const speed_t speed;
extern int mqtt_msg;
extern int mqtt_pub_err;

extern int mqtt_pub(char *topic, char *topicVal);
//extern int nrfReadPayload( uint8_t *payload, int len );
//extern uint8_t nrfRegWrite( int reg, int val);
//extern uint8_t nrfRegRead( int reg );

//************  Forward Declarations
//void parsePayload( void *pvParameters );
extern void parsePayload(void *pvParameters);
extern uint8_t spiXfer( uint8_t *buf, int cnt );
extern uint8_t nrfRegRead( int reg );
extern uint8_t nrfRegWrite( int reg, int val );
extern void nrfPrintDetails(void);
extern int nrfAvailable( uint8_t *pipe_num );
extern int nrfReadPayload( uint8_t *payload, int len );
extern uint8_t nrfFlushTx( void );
extern uint8_t nrfFlushRx( void );
extern int nrfAddrRead( uint8_t reg, uint8_t *buf, int len );
extern uint8_t nrfReadRxPayloadLen(void);



#endif  // __LOFI_H__
