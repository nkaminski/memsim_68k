/* Motorola 68000 memory bus simulator for the Arduino Mega 2560. Fully supports emulation of byte and word size reads and writes.
 Written by Nash Kaminski, BSCPE '16
 
 Wiring: D0-D7 -> Pins 22-29
         D8-D15 -> Pins 37-30
         A1-A10 -> Pins 49-40
         *DTACK -> Pin 2
         *AS -> Pin 3
         *LDS -> Pin 4
         *UDS -> Pin 5
         E(32khz clock) -> 6
         R/*W -> Pin 7
 */

#define DTACK 2
#define AS 3
#define LDS 4
#define UDS 5
#define E 6
#define RW 7
#define SerComm Serial

#define MAXLEN 16

#define is_even(x) (~(x) & 0x01)
#define is_odd(x) ((x) & 0x01)

char inbuf[MAXLEN];
unsigned char counter = 0;
long watchdog;

void set_address(uint16_t addr) {
  PORTL = (addr & 0xff); //A1-A8
  PORTG &= 0b11111100;
  PORTG |= ((addr >> 8) & 0b00000011); //A9-A10
}


uint16_t read_word(uint16_t address) {
  uint16_t data = 0x0000;
  DDRA = 0x00; // data lines to input
  DDRC = 0x00;
  digitalWrite(RW, HIGH); // read
  set_address(address);
  digitalWrite(AS, LOW); // assert as, lds and uds
  digitalWrite(LDS, LOW);
  digitalWrite(UDS, LOW);
  watchdog = millis();
  while (digitalRead(DTACK)) {
    if (millis() > watchdog + 100) { //wait 100ms for DTACK
      SerComm.println("BUS ERROR");
      break;
    }
  }
  data |= PORTA;  //capture data
  data |= (PORTC << 8);
  // Negate as, lds and uds
  digitalWrite(LDS, HIGH);
  digitalWrite(UDS, HIGH);
  digitalWrite(AS, HIGH);
  return data;
}


void write_word(uint16_t address, uint16_t data) {
  digitalWrite(RW, LOW); // write
  DDRA = 0xFF; // data lines to output
  DDRC = 0xFF;

  set_address(address);
  digitalWrite(AS, LOW); // assert as

  // Apply data
  PORTA = (data & 0xff);
  PORTC = (data >> 8) & 0xff;
  //apply data strobes
  digitalWrite(LDS, LOW);
  digitalWrite(UDS, LOW);
  watchdog = millis();
  while (digitalRead(DTACK)) {
    if (millis() > watchdog + 100) { //wait 100ms for DTACK
      SerComm.println("BUS ERROR");
      break;
    }
  }

  // Negate as, lds and uds
  digitalWrite(LDS, HIGH);
  digitalWrite(UDS, HIGH);
  digitalWrite(AS, HIGH);
  PORTA = 0x00; //remove data
  PORTC = 0x00;
  DDRA = 0x00; // data lines back to input
  DDRC = 0x00;
}

uint8_t read_byte(uint16_t address) {
  uint8_t data = 0x00;
  DDRA = 0x00; // data lines to input
  DDRC = 0x00;
  digitalWrite(RW, HIGH); // read
  set_address(address);  // Apply address
  digitalWrite(AS, LOW); // assert AS
  //LDS if odd address, UDS if even. Remember complimentary logic ;)
  digitalWrite(LDS, is_even(address));
  digitalWrite(UDS, is_odd(address));
  //Reset WDT
  watchdog = millis();
  while (digitalRead(DTACK)) {
    if (millis() > watchdog + 100) { //wait 100ms for DTACK
      SerComm.println("BUS ERROR");
      break;
    }
  }
  if (is_even(address)) {
    // Read upper data lines
    data = PORTC;
  }
  else {
    //Read lower data lines
    data = PORTA;
  }

  // Negate as, lds and uds
  digitalWrite(LDS, HIGH);
  digitalWrite(UDS, HIGH);
  digitalWrite(AS, HIGH);
  return data;
}

void write_byte(uint16_t address, uint8_t data) {
  digitalWrite(RW, LOW); // write
  DDRA = 0xFF; // data lines to output
  DDRC = 0xFF;
  set_address(address);  // Apply address
  digitalWrite(AS, LOW); // assert AS
  //Apply data
  PORTA = data;
  PORTC = data;
  //LDS if odd address, UDS if even. Remember complimentary logic ;)
  digitalWrite(LDS, is_even(address));
  digitalWrite(UDS, is_odd(address));
  //Reset WDT
  watchdog = millis();
  while (digitalRead(DTACK)) {
    if (millis() > watchdog + 100) { //wait 100ms for DTACK
      SerComm.println("BUS ERROR");
      break;
    }
  }

  // Negate as, lds and uds
  digitalWrite(LDS, HIGH);
  digitalWrite(UDS, HIGH);
  digitalWrite(AS, HIGH);
  PORTA = 0x00; //remove data
  PORTC = 0x00;
  DDRA = 0x00; // data lines back to input
  DDRC = 0x00;
}

void setup() {
  TCCR4B &= ~(uint8_t)7; // 32KHZ enable clock
  TCCR4B |= CS00;
  DDRL = 0xFF; //Lower address lines
  DDRG |= 0b00000011; //upper address lines
  PORTL = 0x00;
  PORTG &= 0b11111100; //Set address to 0x00
  DDRA = 0x00; //Lower data lines
  DDRC = 0x00; //Upper data lines
  PORTA = 0x00; //high-z on both
  PORTC = 0x00;
  digitalWrite(RW, HIGH);
  digitalWrite(AS, HIGH);
  digitalWrite(LDS, HIGH);
  digitalWrite(UDS, HIGH);
  pinMode(DTACK, INPUT);
  digitalWrite(DTACK, HIGH); //Enable DTACK pullup
  pinMode(AS, OUTPUT);
  pinMode(RW, OUTPUT);
  pinMode(LDS, OUTPUT);
  pinMode(UDS, OUTPUT);
  pinMode(E, OUTPUT);
  analogWrite(E, 127); //50% duty cycle clock
  memset(inbuf, 0x00, MAXLEN);
  SerComm.begin(9600);
}
void comm_reset() {
  counter = 0;
  memset(inbuf, 0x00, MAXLEN);
}
void loop() {
  uint16_t inaddr, indata;
  if (SerComm.available()) {
    inbuf[counter] = SerComm.read();
    counter++;
    if (counter == MAXLEN) {
      SerComm.println("WHAT?");
      comm_reset();
    }
    if (inbuf[counter - 1] == '\n') {
      inaddr = 0x0000;
      indata = 0x0000;
      if (inbuf[0] == 'R') {
        if (counter != 8) {
          SerComm.println("WHAT?");
        } else {
          if (inbuf[1] == 'B') {
            SerComm.println("READ BYTE");
            inaddr = atoi(inbuf + 3);
            SerComm.print("ADDRESS ");
            SerComm.println(inaddr, HEX);
            indata = read_byte(inaddr);
            SerComm.print("BYTE DATA ");
            SerComm.println(indata, HEX);
          } else if (inbuf[1] == 'W') {
            SerComm.println("READ WORD");
            inaddr = atoi(inbuf + 3);
            SerComm.print("ADDRESS ");
            SerComm.println(inaddr, HEX);
            indata = read_word(inaddr);
            SerComm.print("WORD DATA ");
            SerComm.println(indata, HEX);
          } else{
            SerComm.println("WHAT?");
          }
        }
        
      }
      if (inbuf[0] == 'W') {
        if (inbuf[1] == 'B') {
         if (counter != 10) {
            SerComm.println("WHAT?");
          } else {
            SerComm.println("WRITE BYTE");
            inbuf[7] = 0x00;
            inaddr = atoi(inbuf + 3);
            indata = inbuf[8];
            SerComm.print("ADDRESS ");
            SerComm.println(inaddr, HEX);
            SerComm.print("BYTE DATA ");
            SerComm.println(indata, HEX);
            write_byte(inaddr, indata);
          }
        } else if (inbuf[1] == 'W') {
          if (counter != 11) {
            SerComm.println("WHAT?");
          } else {
            SerComm.println("WRITE WORD");
            inbuf[7] = 0x00;
            inaddr = atoi(inbuf + 3);
            indata = inbuf[8];
            indata |= (inbuf[9] << 8);
            SerComm.print("ADDRESS ");
            SerComm.println(inaddr, HEX);
            SerComm.print("WORD DATA ");
            SerComm.println(indata, HEX);
            write_word(inaddr, indata);
          }
        }
      }
      comm_reset();
    }
  }
}
