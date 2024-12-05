#include <Arduino.h>

#define SET_OE(state)     bitWrite(PORTB, 0, state)   // must be high for write process, sets B0 to state
#define SET_WE(state)     bitWrite(PORTB, 1, state)   // must be a 100-1000ns low pulse, sets B1 to state
#define READ_DATA         (((PIND & 0b01111100) << 1) | (PINC & 0b00000111)) // read data from D2-6 and C0-2
#define LED(state)        bitWrite(PORTB, 5, state)   // Indicator LED (waiting indicator)

int state = 0;                    // state machine of Arduino programmer
long writesize = 0;               // total size of data write in bytes
long flashsize = 0x100000;        // 128KB SST39SF010a
long wordsize = 0x8;              // 1 byte / 8-bit word
long addrsize = 0x20000;          // 128K word address space
bool rw = 0;                      // read/write flag
                                  // 0 = read, 1 = write

// 74595 shift register: B2=SER, B3=SRCLK, B4=RCLK
// SER = Serial Data Input, SRCLK = Shift Register Clock Input, RCLK = Storage Register Clock Input

// SST39SF0x0 FLASH: C0-2 = DQ0-2, C3-5 = A16-18
// /OE = Output Enable, /WE = Write Enable

void setup()
{
  PORTB = 0b00000011;               // B5: LED off, /WE=HIGH, /OE=HIGH, B2-4: 74595 SER, SRCLK, RCLK
  DDRB = 0b00111111;                // set all bits to outputs
                                    // Ports            B0: /OE, B1: /WE, B2-4: 74595 SER, SRCLK, RCLK, B5: LED
                                    // Arduino Pinout   D8       D9       D10-12      D10  D11    D12   D13
  PORTC = 0; DDRC = 0b00111000;     // set outputs                      C3-5: address lines A16, A17, A18
                                    // Ports            C0-2: DQ0-2     C3-5: A16-18
                                    // Arduino Pinout   A0-2            A3-5
  PORTD = 0; DDRD = 0b00000000;     // unused D0-1: RX/TX 
                                    // Ports            D2-6: DQ3-7
                                    // Arduino Pinout   D2-6
  Serial.begin(115200, SERIAL_8N1); // SERIAL_8N1 = 8 data bits, no parity, 1 stop bit (default)
}

void loop()
{
  switch(state)
  {
    default: // waiting for handshake 'a'
    {
      if (Serial.available() > 0)
      {
        if (Serial.read() == 'a')  // confirm first handshake
        {
          Serial.write('A');
          LED(HIGH);
          state = 1;
        }
        else state = 0;
      }
      break;
    }
    case 1: // waiting for 'r' or 'w'
    {
      if (Serial.available() > 0)
      {
        char c = Serial.read();
        if (c == 'r') { rw = 0; Serial.write('R'); state = 5; break; }
        else if (c == 'w') { rw = 1; Serial.write('W'); state = 2; break; }
        else state = 0;
      }
      break;
    }
    case 2: // waiting for bytesize and handshake 'b'
    {
      if (Serial.available() > 0)
      {
        char c = Serial.read();
        if (c >= '0' && c <= '9') { writesize = writesize*10 + c - '0'; Serial.write(c); } // echo
        else if (c == 'b') { Serial.write('B'); state = 3; break; } // confirm received bytesize
        else state = 0;
      }
      break;
    }  
    case 3: // receiving and writing data to FLASH
    {
      LED(LOW);
      if (EraseFLASH() == true) // completely erase the FLASH IC first
      {
        Serial.write('C');
        long adr = 0; // always commence writing at address zero
        do
        {
          byte chunk[32];
          int p=0;
          long lastmillis = millis();
          do
          {
            if (Serial.available() > 0)
            {
              chunk[p++] = Serial.read(); lastmillis = millis();
            }
          } while (p < 32 && millis() - lastmillis < 500);

          if (p > 0)                                 // received some data?
          {
            for(int i=0; i<p; i++) WriteFLASH(adr++, chunk[i]);
            Serial.write('D');
          }
        } while (adr < writesize);
        state = 4;
      } else state = 0;
      break;
    }
    case 4: // readout FLASH and send back the data for verification
    {
      LED(HIGH);
      SetPinsToRead();
      SET_OE(LOW);                                    // activate EEPROM outputs
      for(long i=0; i<writesize; i++)
      {
        SetAddress(i);
        Serial.write(READ_DATA);
      }
      SET_OE(HIGH);                                   // deactivate EEPROM outputs
      LED(LOW);
      state = 0;
      break;
    }
    case 5: // readout FLASH in human readable format
    {
      LED(HIGH);

      // Set output and control lines
      SetPinsToRead();
      SET_OE(LOW);                                    // activate FLASH output
      
      Serial.println("-------------------------------------------------");
      Serial.println("Reading FLASH");

      byte old[16];
      byte data[16];
      bool repeats = false;

      // Advance 16 bytes at a time
      for(long base = 0; base < addrsize; base += 0x10)
      {
        // Read 16 bytes of data
        for (byte offset = 0x0; offset < 0x10; offset += 0x1) {
          SetAddress(base + offset);
          data[offset] = READ_DATA;
        }

        bool allZero = (memcmp(old, data, 16) == 0); // check if any difference in data between this line and last read
        // Always print first line or if there is any difference
        if (!allZero || base == 0) {
         // Print base address and 16 bytes of data
         char buf[80];
         sprintf(buf, "%08lx:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
                 base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
                 data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

         Serial.println(buf);

         memcpy(old, data, sizeof(data));
         repeats = false;
        } else {
          // If first difference, print ellipsis
          if (repeats == false) {
            Serial.println("...");
            repeats = true;
          }
        }
      }

      Serial.println("-------------------------------------------------");
      Serial.println("End of FLASH");

      // Set output and control lines
      SET_OE(HIGH);                                   // deactivate flash output

      LED(LOW);
      state = 0;
      break;
    }
  }
}

void SetAddress(long adr)
{ 

  for (byte i = 0x0; i < 0x10; i += 0x1) {
    bitWrite(PORTB, 2, adr & 1); adr = adr>>1;            // push each bits (LSB first) into SER
    bitWrite(PORTB, 3, LOW); bitWrite(PORTB, 3, HIGH);    // toggle SRCLK
  }
  bitWrite(PORTB, 4, HIGH); bitWrite(PORTB, 4, LOW);      // toggle RCLK

  // write the topmost bits
  bitWrite(PORTC, 3, adr & 1); adr = adr>>1;
  bitWrite(PORTC, 4, 0);
  bitWrite(PORTC, 5, 0);
  // bitWrite(PORTC, 4, adr & 1); adr = adr>>1;
  // bitWrite(PORTC, 5, adr & 1); adr = adr>>1;
}

void SetPinsToRead()
{
  DDRC &= 0b00111000;       // set C0-2 to input, C3-5 to output
  PORTC &= 0b00111000;      // switch off C0-2 pull-ups
  DDRD &= 0b10000011;       // set D2-6 to inputs
  PORTD &= 0b10000011;      // switch off D2-6 pull-ups
}

void SetPinsToWrite(byte data)
{
  DDRC |= 0b00000111;       // set C0..2 to outputs
  PORTC = (PORTC & 0b00111000) | (data & 0b00000111); // write the lower 3 bits to C0-2
  DDRD |= 0b01111100;       // set D2..6 to outputs
  PORTD = (PORTD & 0b10000011) | ((data & 0b11111000) >> 1);  // write the upper 5 bits to D2-6
}

bool EraseFLASH()
{
  SET_OE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0xaa); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);   // invoke 'Chip Erase' command 
  SetAddress(0x2aaa); SetPinsToWrite(0x55); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0x80); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0xaa); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x2aaa); SetPinsToWrite(0x55); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0x10); SET_WE(HIGH); SET_WE(LOW); SET_WE(HIGH);
  SetPinsToRead();
  SET_OE(LOW);
  int c = 0; while ((READ_DATA & 128) != 128 && c < 2000) { c++; delayMicroseconds(100); }
  SET_OE(HIGH);
  return c < 2000; // SUCCESS condition
}

bool WriteFLASH(long adr, byte data)
{
  SET_WE(HIGH);
  SET_OE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0xaa); SET_WE(HIGH); SET_WE(LOW); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x2aaa); SetPinsToWrite(0x55); SET_WE(HIGH); SET_WE(LOW); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(0x5555); SetPinsToWrite(0xa0); SET_WE(HIGH); SET_WE(LOW); SET_WE(LOW); SET_WE(HIGH);
  SetAddress(adr); SetPinsToWrite(data); SET_WE(HIGH); SET_WE(LOW); SET_WE(LOW); SET_WE(HIGH);
  SetPinsToRead();
  SET_OE(LOW);              // activate the output for data polling
  int c = 0; while (((READ_DATA&128) != (data&128)) && (c < 100)) c++;   // success < 17
  SET_OE(HIGH);             // deactivate the outputs
  return c < 100;           // SUCCESS condition
}
