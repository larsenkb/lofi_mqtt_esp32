#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "lofi.h"

#define TBUF_LEN        80
#define TOPIC_LEN       40
#define TOPIC_VAL_LEN   40

#if EN_TRANSLATE
char *nodeMap[] = {
  "node/0",
  "node/1",
  "node/2",
  "node/3",
  "node/4",
  "door/GarageN",     // node/5
  "node/6",
  "node/7",
  "door/Hall",    // node/8
  "node/9",
  "node/10",
  "node/11",
  "node/12",
  "node/13",
  "door/Garage",      // node/14
  "node/15",
  "node/16",
  "door/GarageS",     // node/17
  "door/Sliding",     // node/18
  "door/Back",        // node/19
  "node/20",
  "window/OfficeN",   // node/21
  "window/LivingE",   // node/22
  "node/23",
  "window/OfficeS",   // node/24
  "door/GarageS",     // node/25
  "window/MasterE",   // node/26
  "node/27",          // node/27
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
#else
int maxNodes = 100;
#endif

const bool longStr = false;
const bool printPayload = false;
const bool printSeq = false;

char *itoas(int n);
void ftoa(float n, char* res, int afterpoint);

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

    nrfReadPayload( payload, PAYLOAD_LEN );
    nrfRegWrite( NRF_STATUS, 0x70 ); // clear the interrupt

    if (printPayload) {
      dbug_printf("Payload: %02X %02X %02X %02X %02X\n", payload[0], payload[1], payload[2], payload[3], payload[4]);
      goto check;
    }

    // Second byte of payload will never be zero
    if (pl->sensorId == 0 || pl->sensorId > SENID_AHUMD)    // payload[1] == 0)
      goto check;

    nodeId = pl->nodeId;    //payload[0];

	  if (nodeId < 1 || nodeId >= maxNodes) {
		  dbug_printf("Bad nodeId: %d\n", nodeId);
      goto check;
	  }

    sensorId = pl->sensorId;
    seq = pl->seq;

    topicVal[0] = '\0';
    strcpy(topic, "lofi/");
#if EN_TRANSLATE
    strcat(topic, nodeMap[nodeId]);
#else
    strcat(topic, "node/");
    strcat(topic, itoas(nodeId));
#endif

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
      //sprintf(topicVal, "%d.%d", val1/256, val1&0xff);
      strcat(topicVal, itoas(val1/256));
      strcat(topicVal, ".");
      strcat(topicVal, itoas(val1&0xff));
#if LONG_STR
      if (longStr) {
			  tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Rev: %d.%d", val1/256, val1&0xff);
			}
#endif
			break;
		case SENID_CTR:
      strcat(topic, "/ctr");
      //sprintf(topicVal, "%d", val);
      strcat(topicVal, itoas(val));
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Ctr: %4d", val);
			}
#endif
			break;
		case SENID_SW1:
      strcat(topic, "/sw1");
      //sprintf(topicVal, "{\"state\":\"%s\",\"trig\":\"%s\"}", (pl->closed) ? "OPEN" : "SHUT", (pl->trig) ? "PC" : "WD");
      strcat(topicVal, "{\"state\":");
      strcat(topicVal, (pl->closed) ? "\"OPEN\",\"trig\":" : "\"SHUT\",\"trig\":");
      strcat(topicVal, (pl->trig) ? "\"PC\"}" : "\"WD\"}");
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW1: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
#endif
			break;
		case SENID_SW2: // deprecated
      strcat(topic, "/sw1");
      //sprintf(topicVal, (pl->closed) ? "OPEN" : "SHUT");
      strcat(topicVal, (pl->closed) ? "OPEN" : "SHUT");
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  SW2: %s", (pl->closed) ? "OPEN" : "SHUT");
			}
#endif
			break;
		case SENID_VCC:
      strcat(topic, "/vcc");
      //sprintf(topicVal, "%4.2f", (1.1 * 1024.0)/(float)val);
      ftoa((1.1 * 1024.0)/(float)val, topicVal, 2);
#if LONG_STR
			if (longStr) {
				tbufIdx += snprintf(&tbuf[tbufIdx], TBUF_LEN-tbufIdx, "  Vcc: %4.2f",(1.1 * 1024.0)/(float)val);
			}
#endif
			break;
		case SENID_TEMP:
      strcat(topic, "/temp");
      //sprintf(topicVal, "%4.2f", 1.0 * (float)val - 260.0);
      ftoa(1.0 * (float)val - 260.0, topicVal, 2);
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
      //sprintf(topicVal, "%4.1f", ftemp);
      ftoa(ftemp, topicVal, 1);
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
      //sprintf(topicVal, "%4.1f", fhumd);
      ftoa(fhumd, topicVal, 1);
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

#if 0
// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
#endif
  
// reverse:  reverse string s in place
static void reverse(char s[])
{
  int i, j;
  char c;

  for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
    c = s[i];
    s[i] = s[j];
    s[j] = c;
  }
}  

static char s[10];
// itoa:  convert n to characters in s
char *itoas(int n)
{
  int i, sign;

  if ((sign = n) < 0)  /* record sign */
    n = -n;          /* make n positive */
  i = 0;
  do {       /* generate digits in reverse order */
    s[i++] = n % 10 + '0';   /* get next digit */
  } while ((n /= 10) > 0);     /* delete it */
  if (sign < 0)
    s[i++] = '-';
  s[i] = '\0';
  reverse(s);
  return s;
}  

// itoa:  convert n to characters in s
static void itoa(int n, char s[])
{
  int i, sign;

  if ((sign = n) < 0)  /* record sign */
  n = -n;          /* make n positive */
  i = 0;
  do {       /* generate digits in reverse order */
    s[i++] = n % 10 + '0';   /* get next digit */
  } while ((n /= 10) > 0);     /* delete it */
  if (sign < 0)
    s[i++] = '-';
  s[i] = '\0';
  reverse(s);
}

// Converts a given integer x to string str[]. 
// d is the number of digits required in the output. 
// If d is more than the number of digits in x, 
// then 0s are added at the beginning.
static int intToStr(int x, char str[], int d)
{
  int i, sign;

    if ((sign = x) < 0)  /* record sign */
        x = -x;          /* make n positive */
    i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    if (sign < 0)
        str[i++] = '-';
    str[i] = '\0';
    reverse(str);
//    str[i] = '\0';
    return i;
}

static char tfrac[6];
static int pwr[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    if (afterpoint > 6) afterpoint = 6;
    if (afterpoint < 0) afterpoint = 0;

    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
   // itoa(ipart, res);
    intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        strcat(res, ".");
        if (fpart < 0.0)
            fpart = -1.0 * fpart;
        //res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter 
        // is needed to handle cases like 233.007
        fpart = fpart * pwr[afterpoint];

        intToStr((int)fpart, tfrac, afterpoint);
        strcat(res, tfrac);
    }
}
