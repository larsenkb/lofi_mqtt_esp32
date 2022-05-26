
#include "lofi.h"


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
  
//  nrfRegWrite( NRF_STATUS, 0x70 ); // clear the interrupt

  return 0;
}


void nrfPrintDetails(void)
{
  uint8_t   buf[6];

  dbug_puts("================ SPI Configuration ================" );
  dbug_printf("CSN Pin  \t = Custom GPIO%d\n", nrfCSN  );
  dbug_printf("CE Pin  \t = Custom GPIO%d\n", nrfCE );
  dbug_puts("Clock Speed\t = " );
  dbug_puts("1 Mhz");
  dbug_puts("================ NRF Configuration ================");
 

  dbug_printf("STATUS: %02X\n", nrfRegRead( NRF_STATUS ));
  nrfAddrRead( NRF_RX_ADDR_P0, buf, 5 );
  dbug_printf("RX_ADDR_P0: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//  printf("RX_ADDR_P0: %02X\n", nrfRegRead( NRF_RX_ADDR_P0 ));
  nrfAddrRead( NRF_RX_ADDR_P1, buf, 5 );
  dbug_printf("RX_ADDR_P1: %02X%02X%02X%02X%02X\n", buf[1], buf[2], buf[3], buf[4], buf[5]);
//  printf("RX_ADDR_P1: %02X\n", nrfRegRead( NRF_RX_ADDR_P1 ));
  dbug_printf("RX_ADDR_P2: %02X\n", nrfRegRead( NRF_RX_ADDR_P2 ));
  dbug_printf("RX_ADDR_P3: %02X\n", nrfRegRead( NRF_RX_ADDR_P3 ));
  dbug_printf("RX_ADDR_P4: %02X\n", nrfRegRead( NRF_RX_ADDR_P4 ));
  dbug_printf("RX_ADDR_P5: %02X\n", nrfRegRead( NRF_RX_ADDR_P5 ));
//  printf("TX_ADDR: %02X\n", nrfRegRead( NRF_TX_ADDR ));
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
