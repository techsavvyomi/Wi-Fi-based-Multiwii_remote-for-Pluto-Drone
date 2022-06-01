
/* 
 * ESP based Pluto Drone Controller Ver 0.1
 * Omkar M. Dandekar (Drona Aviation Pvt Ltd)
 * 
 * Hardware: Currently tested on Nodemcu
 *           for controller we need two Dual Axis joystick, 4 switches
 *           OLED Display
 */


//---------Macros---------------//
    #include <ESP8266WiFi.h>
    #include <Joystick.h>
    #define MSP_SET_RAW_RC           200
    #define MSP_SET_COMMAND          217
    #include <Wire.h>               
    #include "SSD1306Wire.h" 
    SSD1306Wire display(0x3c, SDA, SCL);     
    // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins) 
     /*
        OLED Display 
        D1 = SCL 
        D2 = SDA
        VCC = 3.3v 
        GND = GND
    */

    #define SCREEN_WIDTH 128                // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 
    #define S0 D5                           /* Assign Multiplexer pin S0 connect to pin D5 of NodeMCU */
    #define S1 D6                           /* Assign Multiplexer pin S1 connect to pin D6 of NodeMCU */                          /* Assign Multiplexer pin S2 connect to pin D2 of NodeMCU */
    #define S3 D3                           /* Switch assign for ARM */
    #define S4 D4                           /* Switch assign for DISARM */
    #define DEV_ON D7       
    #define DEV_OFF D8                    /* LED / Buzzer Optional */
    int decimal = 2;                        // Decimal places of the sensor value outputs 
    int sensor0;                            /* Assign the name "sensor0" as analog output value from Channel C0 */
    int sensor1;                            /* Assign the name "sensor1" as analog output value from Channel C1 */
    int sensor2;                            /* Assign the name "sensor2" as analog output value from Channel C2 */
    int sensor3;                            /* Assign the name "sensor3" as analog output value from Channel C3 */
    uint8_t checksum;
    int arm_stat=2000;
                   

//-----------Typedef Enumuration------------------//
enum
{
    rc_Roll = 0, rc_Pitch, rc_Yaw, rc_Throttle, rcAux1, rcAux2, rcAux3, rcAux4,
};

enum
{
    RC_MAX = 1900,  RC_MID = 1500, RC_MIN = 1100,
};

//----------Pluto Wifi Configuration-----------//

const char* ssid     = "PlutoX_2022_1031"; //Drone SSID 
const char* password = "3053plutox";   //Drone password
const char* host = "192.168.4.1"; //DNS IP address
const uint16_t port = 23; //Port number for pluto server 
WiFiClient client;
//---------------------------------------------//

//----------Rough Stuff------------------------//
bool flag=1;
int count=0;
int co=0;
//---------------------------------------------//

void setup() {
    display.init();
    display.flipScreenVertically();
    delay(2000);
    display.clear();
    Serial.begin(115200);
    pinMode(S0,OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin SO */        
    pinMode(S1,OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S1 */  
    pinMode(DEV_ON,INPUT);
    pinMode(DEV_OFF,INPUT);                       
    pinMode(SIG, INPUT);                      /* Define analog signal pin as input or receiver from the Multiplexer pin SIG */  
    pinMode(S4,INPUT_PULLUP);  
    pinMode(S3,INPUT_PULLUP); 
        // We start by connecting to a WiFi network
        //MSP_disarm();
    Serial.println("Drona Aviation Pvt Ltd");
    Serial.println("Esp Controller");
    Serial.print("Connecting to ");
    Serial.println(ssid);
    //
    
    
     WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    digitalWrite(S2,LOW);
    display_lcd();
}

void loop() {

  if (!client.connect(host, port)) 
  {
       Serial.println("connection failed");
       not_connected_lcd();
       delay(5000);
       return; 
  }
 

Serial.print("Count");Serial.println(co);
/*
  //unneccessary 
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);
*/

int butt=digitalRead(S4);
int butt1=digitalRead(S3);
if(butt==0){
    arm_stat=1500;
    arm_lcd();
    co++;
    digitalWrite(S2,HIGH);
}
if(butt1==0){
    arm_stat=2000;
    disarm_lcd();
    digitalWrite(S2,LOW);
}
     digitalWrite(S0,LOW); digitalWrite(S1,LOW);
    sensor0 = analogRead(SIG);
  
    digitalWrite(S0,HIGH); digitalWrite(S1,LOW); 
    sensor1 = analogRead(SIG);
    
    digitalWrite(S0,LOW); digitalWrite(S1,HIGH);
    sensor2 = analogRead(SIG);
  
    digitalWrite(S0,HIGH); digitalWrite(S1,HIGH); 
    sensor3 = analogRead(SIG);
 

 rcState();   //fuction to send joystic value to msp commands
}

void rcState()
{
    uint32_t init_t = millis();
    static uint32_t preTime = 0;
    uint16_t rc16[8];
    uint8_t rc8[16];

    if((init_t - preTime) > 5)
    {
        rc16[rc_Roll] = map(sensor1,0, 1023,1000, 2000);          // map your joystic 2 here
        rc16[rc_Pitch] = map(sensor0,0, 1023,1000, 2000);         // map your joystic 2 here
        rc16[rc_Yaw] = map(sensor2,0,1023,1000, 2000);           // map your joystic 1 here
        rc16[rc_Throttle] = map(sensor3,0, 1023,1000, 2000); 
        rc16[rcAux1] = RC_MAX;      //900 < AUX1 < 1300 : MAGHOLD ON 1300 < AUX1 < 1700 : HEADFREE MODE ON
        rc16[rcAux2] = RC_MIN;      // AUX2 = 1500, the User code runs (Developer mode On) //Map Button 1 
        rc16[rcAux3] = RC_MID;      //1300 <AUX3 < 1700 : Alt Hold mode on Outside the above range : Throttle range
        rc16[rcAux4] = arm_stat;    //map(read_val,0, 1023,1000, 2000);  //1300 < AUX4 < 1700 : “ARM” the Drone Outside this range : DISARM the Drone. //Map Button 2 

       Serial.println(rc_Throttle);
//*--------CRC-------------*//  
        checksum = 16^200;
        for (uint8_t i = 0; i < 8; i++)
        {
            rc8[2 * i] = rc16[i] & 0xff;
            rc8[2 * i + 1] = (rc16[i] >> 8) & 0xff;
            checksum = checksum^rc8[2 * i]^rc8[2 * i + 1];
        }
//------------MSP Frame sending--------------//
        client.write('$');
        client.write('M');
        client.write('<');
        client.write(16);
        client.write(MSP_SET_RAW_RC);
        for(uint8_t i = 0; i < 16; i++)
        client.write(rc8[i]);
        client.write(checksum);
        int reading;
        reading=  client.read();
     //   Serial.print("Read;  ");Serial.println(reading);
        preTime = init_t;
    }
}
void t_off(){
  //-------Temp Function---------//
    uint32_t init_t = millis();
    static uint32_t preTime = 0;
    uint16_t command_set[1];

    if((init_t - preTime) > 5){
        client.write('$');
        client.write('M');
        client.write('<');
        client.write(2);
        client.write(MSP_SET_COMMAND);
        client.write('2');
    }

}
void display_lcd(){
    display.setFont(ArialMT_Plain_10);
    display.drawString(15, 0, "PLUTO CONTROLLER");
    display.display();
}

void arm_lcd(){
    display.clear();
    display_lcd();
 //display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(10, 15, "CONNECTED");
    display.setFont(ArialMT_Plain_24);
    display.drawString(10, 30, "ARMED");
    display.display();
}
void disarm_lcd(){
    display.clear();
    display_lcd();
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_24);
    display.setFont(ArialMT_Plain_10);
    display.drawString(10, 15, "CONNECTED");
    display.setFont(ArialMT_Plain_24);
    display.drawString(10, 30, "DISARM");
    display.display();
}

void not_connected_lcd(){
    display.clear();
    display_lcd();
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 20, "DISCONNECTED");
    display.display();
}
void connected_lcd(){
    display_lcd();
  //display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    display.drawString(10, 15, "CONNECTED");
    display.display();
}
