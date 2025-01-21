#define S_RXD 18
#define S_TXD 19

#include <SCServo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const int UDP_PORT_1 = 8888;
const int UDP_PORT_2 = 7777;


const char* ssid = "Oneplus";               // Replace with your WiFi SSID
const char* password = "hello@123";         // Replace with your WiFi password
const char* udpAddress = "192.168.200.67";  // Replace with your laptop's IP address

WiFiUDP udp1;
WiFiUDP udp2;
byte buffer1[4 * 6];
char incomingPacket[255];
char incomingPacket2[255];

SCSCL sc;
SMS_STS st;

float Pos_1=0;
float Pos_2=0;
float Pos_3=0;
float Pos_4=0;
float Pos_5=0;
float Pos_6=0;


void connect_wifi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Wifi Connected");

  udp1.begin(UDP_PORT_1);
  udp2.begin(UDP_PORT_2);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  st.pSerial = &Serial1;
  connect_wifi();
  delay(1000);
}

void loop() {
  int packetSize = udp1.parsePacket();
    if (packetSize) {
      String input;
      int len = udp1.read(incomingPacket, 255);
      if (len > 0) {
        incomingPacket[len] = 0;
        input = String(incomingPacket);
      }
      
    Serial.print("Received packet: ");
    Serial.println(incomingPacket); 

    if (input.startsWith("m"))
    {
      input.trim();
      input.remove(0,2);
      // Find the position of the space between val1 and val2
      int spaceIndex = input.indexOf(' ');  // First space
      String val1String = input.substring(0, spaceIndex);

      int nextSpaceIndex = input.indexOf(' ', spaceIndex + 1);  // Second space
      String val2String = input.substring(spaceIndex + 1, nextSpaceIndex);
      
      spaceIndex = nextSpaceIndex;
      nextSpaceIndex = input.indexOf(' ', spaceIndex + 1);  // Third space
      String val3String = input.substring(spaceIndex + 1, nextSpaceIndex);
      
      spaceIndex = nextSpaceIndex;
      nextSpaceIndex = input.indexOf(' ', spaceIndex + 1);  // Fourth space
      String val4String = input.substring(spaceIndex + 1, nextSpaceIndex);

      spaceIndex = nextSpaceIndex;
      nextSpaceIndex = input.indexOf(' ', spaceIndex + 1);  // Fifth space
      String val5String = input.substring(spaceIndex + 1, nextSpaceIndex);

      spaceIndex = nextSpaceIndex;
      String val6String = input.substring(spaceIndex + 1);  // The last value

      // // Convert to integers
      float sjoint_1 = val1String.toFloat();
      float sjoint_2 = val2String.toFloat();
      float sjoint_3 = val3String.toFloat();
      float sjoint_4 = val4String.toFloat();
      float sjoint_5 = val5String.toFloat();
      float sjoint_6 = val6String.toFloat();

      sjoint_1=(-sjoint_1);
      sjoint_2=(-sjoint_2);
      sjoint_4=(-sjoint_4);
      sjoint_5=(-sjoint_5);

      float fjoint_1=(11.378*(sjoint_1*57.2958)+2048);
      float fjoint_2=(11.378*(sjoint_2*57.2958)+2048);
      float fjoint_3=(11.378*(sjoint_3*57.2958)+2048);
      float fjoint_4=(11.378*(sjoint_4*57.2958)+2048);
      float fjoint_5=(2.844*(sjoint_5*57.2958)+512);
      float fjoint_6=(2.844*(sjoint_6*57.2958)+512);

      int ijoint_1=(int)fjoint_1;
      int ijoint_2=(int)fjoint_2;
      int ijoint_3=(int)fjoint_3;
      int ijoint_4=(int)fjoint_4;
      int ijoint_5=(int)fjoint_5;
      int ijoint_6=(int)fjoint_6;


    // delay(10);
      st.WritePosEx(1, ijoint_1, 500, 0);//Servo(ID1) moves at max speed=1500, moves to position=1000.
      st.WritePosEx(2, ijoint_2, 500, 0);//Servo(ID1) moves at max speed=1500, moves to position=1000.
      st.WritePosEx(3, ijoint_3, 500, 0);//Servo(ID1) moves at max speed=1500, moves to position=1000.
      st.WritePosEx(4, ijoint_4, 500, 0);//Servo(ID1) moves at max speed=1500, moves to position=1000.
      sc.WritePos(5, ijoint_5, 0, 500);//Servo(ID1) moves at max speed=1500, moves to position=1000.
      sc.WritePos(6, ijoint_6, 0, 500);//Servo(ID1) moves at max speed=1500, moves to position=1000.

    }}

  int packetSize2=udp2.parsePacket();
  if (packetSize2) {
      int len2 = udp2.read(incomingPacket2, 255);
      if (len2 > 0) {
        incomingPacket2[len2] = 0;
      }
    Serial.print("Received packet from 2nd channel: ");
    Serial.println(incomingPacket2);

    if(st.FeedBack(1)!=-1 & st.FeedBack(2)!=-1 & st.FeedBack(3)!=-1 & st.FeedBack(4)!=-1){
          Pos_1 = (st.ReadPos(1) - 2048)/(11.378*57.295);
          Pos_2 = (st.ReadPos(2) - 2048)/(11.378*57.295);
          Pos_3 = (st.ReadPos(3) - 2048)/(11.378*57.295);
          Pos_4 = (st.ReadPos(4) - 2048)/(11.378*57.295);

    }
    if(sc.FeedBack(5)!=-1 & sc.FeedBack(6)!=-1){
         Pos_5 = (sc.ReadPos(5) - 512)/(2.844*57.295);
         Pos_6 = (sc.ReadPos(6) - 512)/(2.844*57.295);

    } 
    Serial.print(-Pos_1);
    Serial.print(" ");
    Serial.print(-Pos_2);
    Serial.print(" ");
    Serial.print(Pos_3);
    Serial.print(" ");
    Serial.print(-Pos_4);
    Serial.print(" ");
    Serial.print(-Pos_5);
    Serial.print(" ");
    Serial.println(Pos_6);
    float dataArray[6] = { Pos_1, Pos_2, Pos_3, Pos_4, Pos_5, Pos_6 };
    int arrayLength1 = sizeof(dataArray) / sizeof(dataArray[0]);
    memcpy(buffer1, dataArray, arrayLength1 * sizeof(float));

    udp2.beginPacket(udpAddress, UDP_PORT_2);  // Change "destination_ip_address" to the IP address of the destination
    udp2.write(buffer1, arrayLength1 * sizeof(float));
    udp2.endPacket();
    }


}
