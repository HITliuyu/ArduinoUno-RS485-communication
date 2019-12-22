#include "Arduino.h"
#include <SoftwareSerial.h>
#include "RS485_protocol.h"
#define DEBUG 1
#define MAX485_DE 4

SoftwareSerial sserial (5, 6);  // receive pin, transmit pin
unsigned int last_time, current_time;

// callback routines
void fWrite (const byte what)
{
    // rs485.write (what);
    Serial.write (what);    
}
  
int fAvailable ()
{
    // return rs485.available ();  
    return Serial.available ();  
}

int fRead ()
{
    // return rs485.read ();  
    return Serial.read ();  
}

void send_query()
{
    //construct update msg
    byte msg [] = { 
    1,    // device ID
    4    // fn: read
    };

    // send to slave  
    digitalWrite (MAX485_DE, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg);
    while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
        UCSR0A |= 1 << TXC0;  // mark transmission not complete
    while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
    digitalWrite (MAX485_DE, LOW);  // disable sending
#if DEBUG
    sserial.println("Send Query!");
#endif
}


void setup()
{
    sserial.begin (115200);
    Serial.begin(115200);
    pinMode(MAX485_DE, OUTPUT);  // driver output enable
    last_time = millis();
}  // end of setup

void loop()
{
    current_time = millis();
    if((current_time-last_time)>5000)
    {
        send_query();
        last_time = current_time;
    }
    byte buf [10]={0};
    byte received = recvMsg (fAvailable, fRead, buf, sizeof buf,10);
    if(received != 0)
    {
        sserial.print("Received ");
        sserial.print(received); sserial.print(" bytes: ");
        for (int i = 0; i < received; i++) sserial.print(buf[i]);
        sserial.println();
    }
}
