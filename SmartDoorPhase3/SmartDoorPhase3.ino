//#include <PinChangeInt.h>
//#include <Adafruit_CC3000.h>
//#include <SPI.h>
//#include "utility/debug.h"
//#include "utility/socket.h"
//#include <SimpleTimer.h>

// the timer object
//SimpleTimer timer;

//ir pin (analog 0 is pin 14)
#define IRPIN 24

//Device Name for WiFi
#define DEVICE_NAME "CC3000"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   10

// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  12
#define ADAFRUIT_CC3000_CS    4

// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
//Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
//                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

#define WLAN_SSID       "Plastic Mobile"        // cannot be longer than 32 characters!
#define WLAN_PASS       "2plastic1"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define LISTEN_PORT           23    // What TCP port to listen on for connections.

//Adafruit_CC3000_Server chatServer(LISTEN_PORT);

#define thersholdvalue 110

//define clear or set (used for setting analog interrupt) 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define AnalogInterruptReferenceVoltagePin 14



uint8_t STBY = 15;
//10; //standby

//Motor A
uint8_t PWMA = 18;
//13; //Speed control 
uint8_t AIN1 = 16;
//11; //Direction
uint8_t AIN2 = 17; //Direction

volatile int timerId = -1;
volatile uint8_t detected = 0;
volatile uint8_t verified = 0;

volatile int timerVerifiedId = -1;


#define SW1  13

uint8_t lock_state = 0;



/*
* ISR HANDLER for ANALOG INTERUPT
* This gets called when an object is detected over the IR Sensor
*/
ISR(ANALOG_COMP_vect) // handle pin change interrupt for D0 to D7 here
 {
  if(detected == 0){
    Serial.print("Hand Off "); Serial.println("!");
    detected = 1;
    //timer.deleteTimer(timerId);
    verified = 0;
  //  if(verified == 0)
  //    Serial.println("Not Long Enough");
    
    //verified = 0;
    //timerId==-1;
  }else{
    Serial.print("Hand On ");  Serial.println("!");
    detected = 0;
   verified = 1;
 /*   
 if(verified != 1) {
      if( timerVerifiedId != -1)
         timer.deleteTimer(timerVerifiedId);
      
       timer.deleteTimer(timerId);
       
       timerId = timer.setTimeout(3000, verify);
   
       verified = 0;
    }
    else
    {
      Serial.print("No Timer Started"); Serial.println("!");
    }
    */
  }
 }
 
 /*
void verify() {
  if(detected == 0){
    Serial.println("Verfied");
    verified = 1;
    
    if( timerVerifiedId==-1)
    {
      timerVerifiedId = timer.setTimeout(5000, resetVerify);
    }
    else
    {
     // timer.enable(timerVerifiedId); 
     // timer.restartTimer(timerVerifiedId);
      
       timer.deleteTimer(timerVerifiedId);
       
       timerVerifiedId = timer.setTimeout(3000, resetVerify);
    } 
   // timerId==-1;
    
  }else{
    Serial.println("Not Long Enough");
    verified = 0;
    //timerId==-1;
  }
}  

void resetVerify(){
  if(verified == 1)
   {   
     verified = 0;
     timer.deleteTimer(timerVerifiedId);
     timerVerifiedId = -1;
     Serial.println("Not Verified");
   }
}
*/

void setup() {
  
  
  Serial.begin(115200);
  Serial.println("---------------------------------------");
  
  Serial.println(F("Hello, CC3000!\n")); 

//  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  
  /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
  /* !!! Note the additional arguments in .begin that tell the   !!! */
  /* !!! app NOT to deleted previously stored connection details !!! */
  /* !!! and reconnected using the connection details in memory! !!! */
  /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */  
 /* if (!cc3000.begin(false, true, DEVICE_NAME))
  {
    Serial.println(F("Unable to re-connect!? Did you run the SmartConfigCreate"));
    Serial.println(F("sketch to store your connection details?"));
    while(1);
  }
  if (!cc3000.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    while(1);
  }
  Serial.println("Connecting to SSID");
   if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(5000); // ToDo: Insert a DHCP timeout!
  }  
  */
  /* Display the IP address DNS, Gateway, etc.   
  while (! displayConnectionDetails()) {
    delay(1000);
  }
  // Start listening for connections
  chatServer.begin();
  
  Serial.println(F("Listening for connections..."));
    */
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  
  pinMode(AnalogInterruptReferenceVoltagePin, OUTPUT);
  analogWrite(AnalogInterruptReferenceVoltagePin, thersholdvalue);


  //select Multiplexer output as negative input for comparator
  sbi(ADCSRB, ACME);
  //select A0 000 by clearing all the bits in the ADMUX register
  cbi(ADMUX, MUX2);
  cbi(ADMUX, MUX1);
  cbi(ADMUX, MUX0);
  //turn ADC off so we can use comparator
  cbi(ADCSRA, ADEN);

  // Set up the Analog interrupt status register
 ACSR = 
  (0<<ACD)   |   // Analog Comparator: Enabled
  (0<<ACBG)  |   // Analog Comparator Bandgap Select: AIN0 is applied to the positive input
  (0<<ACO)   |   // Analog Comparator Output: Off
  (1<<ACI)   |   // Analog Comparator Interrupt Flag: Clear Pending Interrupt
  (1<<ACIE)  |   // Analog Comparator Interrupt: Enabled
  (0<<ACIC)  |   // Analog Comparator Input Capture: Disabled
  (0<<ACIS1) | (0<<ACIS0);   // Analog Comparator Interrupt Mode: Comparator Interrupt on Rising Output Edge
  
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  // Switch Setup
  pinMode(SW1, INPUT);
  digitalWrite(SW1, HIGH); // Enable internal pullup
  digitalWrite(STBY, LOW); //enable standby
  
}

uint8_t i;
void loop() {
 
 // uint8_t count;
 // char buffer[10];
 //  size_t length = 1;
 // static char last_value = 0;
 // static char index = 0;
 
  
  //length = Serial.readBytes(buffer, length); 
  
  
  
  if( Serial.available() > 0){
   // Serial.println(analogRead(SW1));
    int k = Serial.read();
    if(verified == 1){  // Check to see if it is the same value      
  //        index++;     
            if(!lock_state)
              LockTheDoor();
            else
              UnlockTheDoor();
            lock_state = !lock_state;
    }
    //last_value = buffer[0];
  }
 // timer.run();
  // Try to get a client which is connected.
 /* Adafruit_CC3000_ClientRef client = chatServer.available();
  if (client) {
     // Check if there is data available to read.
     if (client.available() > 0) {
       // Read a byte and write it to all clients.
       uint8_t ch = client.read();
       chatServer.write(ch);
        if(ch == 'a' && verified == 1){  // Check to see if it is the same value      
            index++;     
              if(!lock_state)
                LockTheDoor();
              else
                UnlockTheDoor();
              lock_state = !lock_state;
      }
     }
  }*/
}
/*************************************************************************/
void move(int speed, int direction){
//Move motor at speed and direction
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
}

/*************************************************************************/
void LockTheDoor(void){
  if(digitalRead(SW1) == LOW){
     digitalWrite(STBY, HIGH); //disable standby
     move(120,0);
     while(digitalRead(SW1) == LOW);
     delay(200);
     move(0,0);
     digitalWrite(STBY, LOW); //enable standby
  }
}
/*************************************************************************/
void UnlockTheDoor(void){
  if(digitalRead(SW1) == HIGH){
     digitalWrite(STBY, HIGH); //disable 
     move(120,1);
     while(digitalRead(SW1) == HIGH);
     delay(200);
     move(0,1);
     digitalWrite(STBY, LOW); //enable standby
  }
}
/*************************************************************************/
/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/*************************************************************************
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
*/
