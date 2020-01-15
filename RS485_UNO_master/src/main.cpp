#include <avr/wdt.h>
#include <SPI.h>
#include "Arduino.h"
#include <SoftwareSerial.h>
#include "RS485_protocol.h"

#define UltrasonicPin A3
#define SS 10 //slave select pin
#define BUFFSIZE 14
#define PWM_PIN_WHITE 3 //pwm output for white led
#define PWM_PIN_YELLOW 9 // pwm output for yellow led
#define FAN_HALL_SENSOR_PIN_0 A0 // read hall sensor value from fan 0 
#define FAN_HALL_SENSOR_PIN_1 A1 // read hall sensor value from fan 1 
#define FAN_HALL_SENSOR_PIN_2 A2 // read hall sensor value from fan 2
#define MAX485_DE 4
#define MAX_SLAVE_NUM 3
#define RE_TRANS_NUM 2
#define QUERY_PERIOD 5000//900000 //time interval to query slaves, 15 mins = 1000 * 60 * 15 = 900,000
#define SENSOR_READ_PERIOD 4000 //time interval to update local sensor reading
#define HALL_TRIGGER LOW //hall sensor trigger signal

#define DEBUG 1

//union definition
union cvtfloat {
    float val;
    unsigned char bytes[4];
} myfloat;
union cvtint {
    int val;
    unsigned char bytes[2];
} myint;
union cvtlong {
    long val;
    unsigned char bytes[4];
} mylong;

enum STATE
{
    SEND_UPDATE,
    WAIT_UPDATE_ACK,
    SEND_QUERY,
    WAIT_QUERY_RESPONSE,
    NORMAL_EXECUTION
};

enum STATE loop_state;
//buffer used to store sensor data
//|0.1.Ultrasonic sensor| 2.3.4.Master Fan Status| 5.6.7. Slave_1 Fan Status| 8.9.10. Slave_2 Fan Status|11.12.13. Slave_3 Fan Status|
uint8_t sensorData [BUFFSIZE];
//hall sensor read variables
uint32_t current_time = 0;
uint32_t elapsed_time = 0;
bool hall_trigger_flag_0 = false;
bool hall_trigger_flag_1 = false;
bool hall_trigger_flag_2 = false;
byte fan_cnt_0 = 0;
byte fan_cnt_1 = 0;
byte fan_cnt_2 = 0;

//SPI control variables
byte command = 0;
int i = 0;
int k = 0;
byte checksum;

//Modbus slave control variables
//{LED white PWM, LED yellow PWM, Fan speed PWM, LED Switch, Fan Switch}
byte slave_control_buffer[5] = {128, 128, 128, 1, 1};
uint16_t slave_active_array[MAX_SLAVE_NUM] = {0};
bool slave_control_buffer_update_flag = true;
//bool slave_id_update_flag = true;
int retransmission_cnt = RE_TRANS_NUM;
int tx_cnt = 0; //count how many slaves has been queried 
byte query_target_device_id = 1;
byte update_target_device_id = 1;
uint32_t last_read_sensor_time = 0;
uint32_t last_query_slave_time = 0;
const byte LED_PIN = 13;

SoftwareSerial sserial (5, 6);  // receive pin, transmit pin

//function prototype declaration
void resetParam();
void watchdogSetup(void);
void send_query();
void send_update();

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

void setup()
{
    sserial.begin (115200);
    Serial.begin(115200);
    pinMode(PWM_PIN_WHITE, OUTPUT);
    pinMode(PWM_PIN_YELLOW, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);  // driver output enable
    pinMode(LED_PIN, OUTPUT);  // built-in LED
    pinMode(FAN_HALL_SENSOR_PIN_0, INPUT_PULLUP);
    pinMode(FAN_HALL_SENSOR_PIN_1, INPUT_PULLUP);
    pinMode(FAN_HALL_SENSOR_PIN_2, INPUT_PULLUP);
    
    //variable initialization
    i = 0;
    checksum = 0;
    // turn on SPI in slave mode
    SPCR |= bit (SPE);
    
    // have to send on master in, *slave out*
    pinMode(MISO, OUTPUT);
    
    // now turn on interrupts
    //  SPI.attachInterrupt();
     SPCR |= _BV(SPIE);
    
     // interrupt for SS falling edge
    //    attachInterrupt (0, resetParam, FALLING);
    attachInterrupt (0, resetParam, RISING);// we reset parameters when SS is released
    //setup watchdog
    watchdogSetup();
    loop_state = SEND_UPDATE;//loop starts from checking slave id
    last_read_sensor_time = millis();
    last_query_slave_time = millis();
  
}  // end of setup

// SPI interrupt routine
ISR (SPI_STC_vect)
{
    byte c = SPDR;  // grab byte from SPI Data Register
    switch (command)
    {
    // no command? then this is the command
    case 0:
      command = c;
      SPDR = 0;
      break;
      
    // transmit buffered data
    case 'a':
      if (i < BUFFSIZE)
      {
          SPDR = sensorData[i];
          checksum += sensorData[i];
          i++;
      }
      else
      {
          SPDR = (checksum & 0xff);      
      } 
      break;

    //change PWM output
    case 'b':
        slave_control_buffer[k] = SPDR;
        if (k >= 4) // k >= sizeof(slave_control_buffer)-1
        {    
            slave_control_buffer_update_flag = true;
        }
        k++;
        break;
    } // end of switch        
}  // end of interrupt routine SPI_STC_vect

//configure watchdog
void watchdogSetup(void)
{
    cli(); // disable all interrupts
    wdt_reset(); // reset the WDT timer
    /*
     WDTCSR configuration:
     WDIE = 1: Interrupt Enable
     WDE = 1 :Reset Enable
     WDP3 = 1 :For 8000ms Time-out
     WDP2 = 0 :For 8000ms Time-out
     WDP1 = 0 :For 8000ms Time-out
     WDP0 = 1 :For 8000ms Time-out
    */
    // Enter Watchdog Configuration mode:
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    // Set Watchdog settings:
    WDTCSR = (0<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
    sei();
} //end of watch dog configuration

void loop()
{
    wdt_reset();

    current_time = millis();
    switch(loop_state)
    {
        case SEND_UPDATE:
        {
            send_update();
            loop_state = WAIT_UPDATE_ACK;
            break;
        }
        case WAIT_UPDATE_ACK:
        {
            byte buf [10]={0};
            byte received = recvMsg (fAvailable, fRead, buf, sizeof(buf), 10); //receive timeout 10ms
            if((received == 0) && retransmission_cnt)
            {
                loop_state = SEND_UPDATE;
                retransmission_cnt--;
            }
            else
            {
#if DEBUG
                sserial.print("Received Update ACK: ");
                sserial.print(received); sserial.print(" bytes:");
                for (int i = 0; i < received; i++) sserial.print(buf[i]);
                sserial.println();
#endif                
                update_target_device_id = (update_target_device_id == MAX_SLAVE_NUM)? 1:(update_target_device_id+1);
                retransmission_cnt = RE_TRANS_NUM;
                loop_state = (update_target_device_id == 1)? NORMAL_EXECUTION:SEND_UPDATE;
            }
            break;
        }
        case NORMAL_EXECUTION:
        {
            //Read local fan status from hall sensors
            //check fan 0 hall sensor
            if (digitalRead(FAN_HALL_SENSOR_PIN_0) == HALL_TRIGGER)
            {
                if(hall_trigger_flag_0 == false)
                {
                    fan_cnt_0++;
                    hall_trigger_flag_0 = true;
                }
            }else
            {
                hall_trigger_flag_0 = false;
            }
            //check fan 1 hall sensor
            if (digitalRead(FAN_HALL_SENSOR_PIN_1) == HALL_TRIGGER)
            {
                if(hall_trigger_flag_1 == false)
                {
                    fan_cnt_1++;
                    hall_trigger_flag_1 = true;
                }
            }else
            {
                hall_trigger_flag_1 = false;
            }
            //check fan 2 hall sensor
            if (digitalRead(FAN_HALL_SENSOR_PIN_2) == HALL_TRIGGER)
            {
                if(hall_trigger_flag_2 == false)
                {
                    fan_cnt_2++;
                    hall_trigger_flag_2 = true;
                }
            }else
            {
                hall_trigger_flag_2 = false;
            }
            
            elapsed_time = current_time - last_read_sensor_time;
                        
            if(elapsed_time>=SENSOR_READ_PERIOD)
            {
                //read analog signal from ultrasonic sensor
                myint.val = analogRead(UltrasonicPin);
                sensorData[0] = myint.bytes[0];
                sensorData[1] = myint.bytes[1];
#if 0
                sserial.println("-----fan cnt-----");
                sserial.println(fan_cnt_0/(SENSOR_READ_PERIOD/1000));
                sserial.println(fan_cnt_1/(SENSOR_READ_PERIOD/1000));
                sserial.println(fan_cnt_2/(SENSOR_READ_PERIOD/1000));
#endif
                sensorData[2] = fan_cnt_0/(SENSOR_READ_PERIOD/1000);
                sensorData[3] = fan_cnt_1/(SENSOR_READ_PERIOD/1000);
                sensorData[4] = fan_cnt_2/(SENSOR_READ_PERIOD/1000);
                fan_cnt_0 = 0;
                fan_cnt_1 = 0;
                fan_cnt_2 = 0;

                last_read_sensor_time = current_time;
#if 1
                //only used for test purpose to frequently update slaves
                slave_control_buffer_update_flag = true;
#endif
            }
            //pwm output setting
            if(slave_control_buffer_update_flag)
            {
                //Local PWM signal update
                analogWrite(PWM_PIN_WHITE, slave_control_buffer[0]);
                analogWrite(PWM_PIN_YELLOW, slave_control_buffer[1]);
                
                //Slave control parameters update
                loop_state = SEND_UPDATE;
                slave_control_buffer_update_flag = false;
                break; //early break to enter SEND_UPDATE, otherwise, it loop_state might be changed in the next if block
            }
            
            //query slaves' fan status
            if((current_time-last_query_slave_time)>=QUERY_PERIOD)
            {
                loop_state = SEND_QUERY;
                last_query_slave_time = current_time;
            }
            break;
        }
        case SEND_QUERY:
        {
            send_query();
            loop_state = WAIT_QUERY_RESPONSE;
            break;
        }
        case WAIT_QUERY_RESPONSE:
        {
            byte buf [10];
            byte received = recvMsg (fAvailable, fRead, buf, sizeof(buf), 10); //receive timeout 10ms
            if((received == 0) && retransmission_cnt)
            {
                loop_state = SEND_QUERY;
                retransmission_cnt--;
            }
            else
            {
#if DEBUG
                sserial.print("Recv query response: ");
                sserial.print(received); sserial.print(" bytes: ");
                if(received){for (int i = 0; i < received; i++) sserial.print(buf[i]);}
                sserial.println();
                sserial.print("retransmission cnt is ");sserial.println(retransmission_cnt);
#endif                
                //update received slave status
                if(retransmission_cnt)
                {
                    for(int i=0; i<3; i++) sensorData[i+2+query_target_device_id*3] = buf[i+2];
                }
                else
                {
                    for(int i=0; i<3; i++) sensorData[i+2+query_target_device_id*3] = 255; // set fan status to ERROR code 255
                }
#if DEBUG
                sserial.println("sensorData array is: ");
                for(int i=0; i<14; i++) {sserial.print(sensorData[i]); sserial.print(" ");}
#endif

                retransmission_cnt = RE_TRANS_NUM;
                query_target_device_id = (query_target_device_id == MAX_SLAVE_NUM)? 1:(query_target_device_id+1);
                loop_state = NORMAL_EXECUTION;
            }
            break;
        }
        default: break;
    }
    
}  // end of loop

void send_update()
{
    //construct update msg
    byte msg [7] = { 
    update_target_device_id,    // device ID (255 means broadcast)
    16,    // fn: write
    slave_control_buffer[0], // write value
    slave_control_buffer[1],
    slave_control_buffer[2],
    slave_control_buffer[3],
    slave_control_buffer[4]
    };

    // send to slave  
    digitalWrite (MAX485_DE, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof(msg));
    while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
        UCSR0A |= 1 << TXC0;  // mark transmission not complete
    while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
    digitalWrite (MAX485_DE, LOW);  // disable sending
#if DEBUG
    sserial.println("Send Update param!");
#endif
    
}

void send_query()
{
    //construct update msg
    byte msg [] = { 
    query_target_device_id,    // device ID
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

void resetParam()
{
    command = 0;
    i = 0;
    k = 0;
    checksum = 0;
}  // end of interrupt service routine (ISR) resetParam
