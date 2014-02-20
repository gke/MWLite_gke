

uint32_t rx32(void) {
  uint32_t t = rx16();
  t += (uint32_t)rx16() << 16;
  return t;
} // rx32

uint16_t rx16(void) {
  uint16_t t = rx8();
  t += (uint16_t)rx8() << 8;
  return t;
} // rx16

uint8_t rx8(void)  {
  return cmdBuf[cmdIndex++]&0xff;
} // rx8

void tx32(uint32_t a) {
  tx8((a    ) & 0xFF);
  tx8((a>> 8) & 0xFF);
  tx8((a>>16) & 0xFF);
  tx8((a>>24) & 0xFF);
} // tx32

void tx16(int16_t a) {
  tx8((a   ) & 0xFF);
  tx8((a>>8) & 0xFF);
} // tx16

void tx8(uint8_t a) {
  uint8_t t = txHead[0];

  if (++t >= TX_BUFFER_SIZE) 
  t = 0;
  txBuff[t][0] = a;
  csum ^= a;
  txHead[0] = t;
} // tx8


ISR(USART1_UDRE_vect) { // Spektrum or GPS
  uint8_t newTail = txTail[1];

  if (txHead[1] != newTail) {
    if (++newTail >= TX_BUFFER_SIZE) 
      newTail = 0;
    UDR1 = txBuff[newTail][1]; 
    txTail[1] = newTail;
  }
  if (newTail == txHead[1]) 
    UCSR1B &= ~(1<<UDRIE1);
} // USART1_UDRE_vect

ISR(USART1_RX_vect)  { 
  store_uart_in_buf(UDR1, 1); 
} // USART1_RX_vect

void uartSendData(uint8_t port) {

  switch (port) {
  case 0:
    while(txHead[0] != txTail[0]) {
      if (++txTail[0] >= TX_BUFFER_SIZE) 
        txTail[0] = 0;
      USB_Send(USB_CDC_TX, txBuff[txTail[0]],1);
    }
    break;
  case 1: 
    UCSR1B |= (1<<UDRIE1); 
    break;
  }
} // uartSendData

static void inline serialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);

  txHead[port] = txTail[port] = rxHead[port] = rxTail[port] = 0;

  switch (port) {
  case 0:  
    UDIEN &= ~(1<<SOFE); 
    break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
  case 1: 
    UCSR1A  = (1<<U2X1); 
    UBRR1H = h; 
    UBRR1L = l; 
    UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); 
    break;
  }
} // serialOpen

static void inline store_uart_in_buf(uint8_t data, uint8_t port) {
  uint32_t nowuS, intervaluS;
  uint8_t newHead;

#if defined(SPEKTRUM)
  if (port == SPEK_SERIAL_PORT) {
    if (!spekFrameSeen) {       
      noInterrupts();
      nowuS = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); // avoid micros() call - slow?
      interrupts(); 

      intervaluS = nowuS - spekLastUpdateuS; 
      spekLastUpdateuS = nowuS;
      if (intervaluS > 5000) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer. 
        rxTail[port] = 0;
        rxHead[port] = 0;
        spekFrameSeen = true;
      }
    }
  }
#endif

  newHead = rxHead[port];
  if (++newHead >= RX_BUFFER_SIZE) 
    newHead = 0;
  if (newHead != rxTail[port]) {
    rxBuff[rxHead[port]][port] = data;  
    rxHead[port] = newHead;
  }
} // store_uart_in_buf

uint8_t serialRead(uint8_t port) {
  uint8_t newTail;
  uint8_t ch;

#if (ARDUINO >= 100)
  if(port == 0) USB_Flush(USB_CDC_TX);
#endif

  if(port == 0) 
    ch = USB_Recv(USB_CDC_RX);  
  else {  
    newTail = rxTail[port];
    ch = rxBuff[newTail][port];
    if (rxHead[port] != newTail) {
      if (++newTail >= RX_BUFFER_SIZE) 
        newTail = 0;
      rxTail[port] = newTail;
    }
  }
  return ch;
} // serialRead

uint8_t serialAvailable(uint8_t port) {

  return (port == 0) ? USB_Available(USB_CDC_RX):
  (rxHead[port] - rxTail[port]) % RX_BUFFER_SIZE;
} // serialAvailable

#if defined(SPEKTRUM)
  uint8_t serialPeek(uint8_t port) {
    uint8_t c = rxBuff[rxTail[port]][port];
    if ((rxHead[port] != rxTail[port])) return c; else return 0;
  }
#endif

void serialWrite(uint8_t port,uint8_t c){
  tx8(c);
  uartSendData(port);
} // serialWrite



























































