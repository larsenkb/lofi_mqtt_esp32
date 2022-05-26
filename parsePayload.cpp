#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "lofi.h"


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
  int pkt_avail = FALSE;
  float fhumd;
  float ftemp;
  bool rv;
  long rcvVal;
      
  sensor_t  *pl = (sensor_t *)payload;
  
  
  for (;;) {
    
    if (!pkt_avail)
      xQueueReceive( nrf_queue, &rcvVal, portMAX_DELAY );
//      xSemaphoreTake( sema_nrf, portMAX_DELAY );

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

#if LONG_STR
	  if (longStr) {
			tbufIdx = snprintf(&tbuf[0], TBUF_LEN, "Id: %2d", nodeId);
	  }

    if (longStr && printSeq) {
      tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Seq: %1d", seq);
    }
#endif

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
      sprintf(topicVal, "%d.%d", val1/256, val1&0xff);
#if LONG_STR
      if (longStr) {
			  tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Rev: %d.%d", val1/256, val1&0xff);
			}
#endif
			break;
		case SENID_CTR:
      strcat(topic, "/ctr");
      sprintf(topicVal, "%d", val);
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Ctr: %4d", val);
			}
#endif
			break;
		case SENID_SW1:
      strcat(topic, "/sw1");
      sprintf(topicVal, "{\"state\":\"%s\",\"trig\":\"%s\"}", (pl->closed) ? "OPEN" : "SHUT", (pl->trig) ? "PC" : "WD");
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW1: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
#endif
			break;
		case SENID_SW2: // deprecated
      strcat(topic, "/sw1");
      sprintf(topicVal, (pl->closed) ? "OPEN" : "SHUT");
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW2: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
#endif
			break;
		case SENID_VCC:
      strcat(topic, "/vcc");
      sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
			}
#endif
			break;
		case SENID_TEMP:
      strcat(topic, "/temp");
      sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Temp: %4.2f",1.0 * (float)val - 260.0);
			}
#endif
			break;
    case SENID_ATEMP:
      strcat(topic, "/atemp");
      ftemp = (((float)(val1 * 200))/0x100000) - 50.0;
      ftemp = (ftemp * 1.8) + 32.0;
      if (ftemp < -30.0) ftemp = -30.0;
      if (ftemp > 140.0) ftemp = 140.0;
      ftemp += 0.05;
      sprintf(topicVal, "%4.1f", ftemp);
#if LONG_STR
      if (longStr) {
        tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  ATemp: %4.1f", ftemp);
      }
#endif
      break;
      
    case SENID_AHUMD:
    
      strcat(topic, "/ahumd");
      fhumd = ((float)(val1 * 100))/0x100000;
      fhumd += 0.05;
      if (fhumd < 0.0) fhumd = 0.0;
      if (fhumd > 100.0) fhumd = 100.0;
      sprintf(topicVal, "%4.1f", fhumd);
#if LONG_STR
      if (longStr) {
        tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  ATemp: %4.1f", fhumd);
      }
#endif
      break;
		default:
			dbug_printf("Bad SensorId: %d\n", sensorId);
			goto check;
      //continue;
			break;
		}

    mqtt_msg++;
    rv = mqtt_pub(topic, topicVal);
    if (!rv) {
      mqtt_pub_err++;
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

check:
    pkt_avail = ((nrfRegRead(NRF_FIFO_STATUS) & 1) == 0);
    
//    printf( "parsePayload MEMORY WATERMARK %d\n", uxTaskGetStackHighWaterMark(NULL) );

  } //endof: for(;;)
  
  // should never get here
  vTaskDelete ( NULL );
}
