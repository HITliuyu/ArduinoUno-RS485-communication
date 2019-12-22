#include <avr/wdt.h>
#include "Arduino.h"
#include <SoftwareSerial.h>
#include "RS485_protocol.h"

#define PWM_PIN_WHITE 3 //pwm output for white led
#define PWM_PIN_YELLOW 9 // pwm output for yellow led
#define PWM_PIN_FAN_SPEED 10 //pwm output for fan speed in extention box
#define FAN_PIN 7 //fan on/off control signal
#define LED_PIN 8 //LED lamp on/off control signal
#define FAN_HALL_SENSOR_PIN_0 A0 // read hall sensor value from fan 0 
#define FAN_HALL_SENSOR_PIN_1 A1 // read hall sensor value from fan 1 
#define FAN_HALL_SENSOR_PIN_2 A2 // read hall sensor value from fan 2
#define SLAVE_ID 2 //Slave ID
#define MAX485_DE 4 //Txen pin
#define CHECK_FAN_PERIOD 5000 //time interval to check fan speed, unit in millisecond
#define HALL_TRIGGER HIGH //hall sensor trigger signal

#define DEBUG 1
enum STATE
{
    NORMAL_EXECUTION,
    RECV_MSG,
    UPDATE_PARAM,
    REPORT_FAN_STATUS
};
enum STATE loop_state;

//Modbus slave control variables
//{LED white PWM, LED yellow PWM, Fan speed PWM, LED Switch, Fan Switch}
byte slave_control_buffer[5] = {250, 250, 250, 0, 0};
byte recv_buf [10] = {0};
byte fan_status[3] = {0};
uint32_t current_time = 0;
uint32_t last_time = 0;
// uint32_t elapsed_time = 0;
bool hall_trigger_flag_0 = false;
bool hall_trigger_flag_1 = false;
bool hall_trigger_flag_2 = false;
byte fan_cnt_0 = 0;
byte fan_cnt_1 = 0;
byte fan_cnt_2 = 0;

SoftwareSerial sserial (5, 6);  // receive pin, transmit pin

//Function prototype declaration
void update_control(void);
void send_ACK();
void send_query_response();

//RS485 callback routines
// callback routines
void fWrite (const byte what)
{
    Serial.write (what);    
}
  
int fAvailable ()
{
    return Serial.available ();  
}

int fRead ()
{
    return Serial.read ();  
}
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

void setup()
{
    sserial.begin (115200);
    Serial.begin(115200);
    pinMode (MAX485_DE, OUTPUT);  // driver output enable
    pinMode(PWM_PIN_WHITE, OUTPUT);
    pinMode(PWM_PIN_YELLOW, OUTPUT);
    pinMode(PWM_PIN_FAN_SPEED, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode (13, OUTPUT);
    pinMode(FAN_HALL_SENSOR_PIN_0, INPUT_PULLUP);
    pinMode(FAN_HALL_SENSOR_PIN_1, INPUT_PULLUP);
    pinMode(FAN_HALL_SENSOR_PIN_2, INPUT_PULLUP);

    loop_state = NORMAL_EXECUTION;
    last_time = millis();
    //initialize functions using default control parameters
    update_control();
}

void loop()
{
    wdt_reset();
    current_time = millis();
    switch (loop_state)
    {
        case NORMAL_EXECUTION:
        {
            
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
            
            // elapsed_time = current_time - last_time;
            //periodically check fan speed by reading hall sensor
            if((current_time - last_time)>=CHECK_FAN_PERIOD)
            {
#if 0
                Serial.println("-----fan cnt-----");
                Serial.println(fan_cnt_0/(CHECK_FAN_PERIOD/1000));
                Serial.println(fan_cnt_1/(CHECK_FAN_PERIOD/1000));
                Serial.println(fan_cnt_2/(CHECK_FAN_PERIOD/1000));
#endif
                fan_status[0] = 31;//fan_cnt_0/(CHECK_FAN_PERIOD/1000);
                fan_status[1] = 32;//fan_cnt_1/(CHECK_FAN_PERIOD/1000);
                fan_status[2] = 33;//fan_cnt_2/(CHECK_FAN_PERIOD/1000);
                fan_cnt_0 = 0;
                fan_cnt_1 = 0;
                fan_cnt_2 = 0;
                last_time = current_time;
            }
            byte buf[10]={0};
            byte received = recvMsg (fAvailable, fRead, buf, sizeof (buf), 10); //timeout set to 10 ms
            if(received)
            {
#if DEBUG       
                sserial.print("receive ");
                sserial.print(received);
                sserial.print(" bytes: ");
                for (int i=0; i<received; i++) sserial.print(buf[i]);
                sserial.println();
#endif
                for (int i=0; i<received; i++) recv_buf[i] = buf[i];
                loop_state = RECV_MSG;
            }
            break;
        }
        case RECV_MSG:
        {
            if(recv_buf[0] != SLAVE_ID)
            {
#if DEBUG
                sserial.println("ID Wrong, not my message");
#endif
                loop_state = NORMAL_EXECUTION;
                break;
            }
            if(recv_buf[1] == 16)
            {
#if DEBUG
                sserial.println("Message fnt is 16");
#endif                
                loop_state = UPDATE_PARAM;
            }
            else if(recv_buf[1] == 4)
            {
#if DEBUG
                sserial.println("Message fnt is 4");
#endif
                loop_state = REPORT_FAN_STATUS;
            }
            else
            {
#if DEBUG
                sserial.println("Unknown Message");
#endif
                loop_state = NORMAL_EXECUTION;
            }
            break;
        }
        case UPDATE_PARAM:
        {
            //update parameters
            for(int i=0; i<5; i++)
            {
                slave_control_buffer[i] = recv_buf[i+2];
            }
#if DEBUG
            sserial.print("Update control param to: ");
            for(int i=0; i<5; i++)
            {
                sserial.print(recv_buf[i+2]);
            }
            sserial.println();
#endif
            update_control();
            //set recv buffer back to 0
            memset(recv_buf, 0, sizeof(recv_buf));
            
            //send Write ACK
            send_ACK();
            loop_state = NORMAL_EXECUTION;
            break;
        }
        case REPORT_FAN_STATUS:
        {
#if DEBUG
            sserial.print("Enter report fan status: ");
            for(int i=0; i<2; i++)
            {
                sserial.print(recv_buf[i]);
            }
            sserial.println("<--recv_buf");
#endif
            //set recv buffer back to 0
            memset(recv_buf, 0, sizeof(recv_buf));
            send_query_response();

#if DEBUG
            sserial.print("Fan status: ");
            for(int i=0; i<=2; i++) sserial.print(fan_status[i]);
            sserial.println();
#endif
            
            loop_state = NORMAL_EXECUTION;
            break;
        }
        default:  break;
    }
}  // end of loop

//apply new control parameters 
void update_control(void)
{
    analogWrite(PWM_PIN_WHITE, slave_control_buffer[0]);
    analogWrite(PWM_PIN_YELLOW, slave_control_buffer[1]);
    analogWrite(PWM_PIN_FAN_SPEED, slave_control_buffer[2]);
    digitalWrite(FAN_PIN, slave_control_buffer[3]);
    digitalWrite(LED_PIN, slave_control_buffer[4]);
}

void send_ACK()
{
    //send Write ACK
    byte msg [3] = {
        0,  // device 0 (master)
        16,  // Fn: 16 (same as write function code)
        10,  //ACK value
    };
    delay (1);  // give the master a moment to prepare to receive
    digitalWrite (MAX485_DE, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg);
    while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
        UCSR0A |= 1 << TXC0;  // mark transmission not complete
    while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
    digitalWrite (MAX485_DE, LOW);  // disable sending
}

void send_query_response()
{
    byte msg [5] = {
        0,  // device 0 (master)
        4,  // Fn: 4 (same as read function code)
        fan_status[0],  //fan status
        fan_status[1],
        fan_status[2]
    };

    delay (1);  // give the master a moment to prepare to receive
    digitalWrite (MAX485_DE, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg);
    while (!(UCSR0A & (1 << UDRE0)))  // Wait for empty transmit buffer
        UCSR0A |= 1 << TXC0;  // mark transmission not complete
    while (!(UCSR0A & (1 << TXC0)));   // Wait for the transmission to complete
    digitalWrite (MAX485_DE, LOW);  // disable sending
}