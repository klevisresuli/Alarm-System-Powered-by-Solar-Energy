#include <SoftwareSerial.h>
#include "DHT.h"
const int DHTPIN=2,DHTPower=4,DHTTYPE = DHT11;
const int BuzzerPin = 3,GasPower = 6,Gas_Pin_Analog = A0,WaterLevelPower = 8,WaterLevelPin = A1;
const int MotionPower = 5,DetectMotion = A2,FirePower = 11,FireDigital = 12,rxPin = 10,txPin = 9;
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial BTSerial(rxPin, txPin); // RX TX
String output_message="",messageBuffer = "",input_message="";
bool detected_start= false;
int DHTSystem=1,GasSystem=1,WaterSystem=1,MotionSystem=1,FireSystem=1;
int Motion_Detected = -1, Fire_Detected= -1,counter=0, current_time=0, prev_time=0;
float Water_Level=-1.00,Gas_Value=-1.00, Temp_Value=-1.00, Humidity_Value=-1.00;
bool DHT_Alarm=false, Gas_Alarm=false, Water_Alarm=false,Triggered=false;
void setup() {
    //CLKPR=0x80;
    //CLKPR=0x01;
  pinMode(DHTPower, OUTPUT);
  pinMode(GasPower, OUTPUT); 
  pinMode(WaterLevelPower, OUTPUT);
  pinMode(MotionPower, OUTPUT);
  pinMode(FirePower, OUTPUT);
  pinMode(FireDigital, INPUT);
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(DHTPower, LOW);
  digitalWrite(GasPower, LOW);
  digitalWrite(WaterLevelPower, LOW);
  digitalWrite(MotionPower, LOW);
  digitalWrite(FirePower, LOW); 
  digitalWrite(BuzzerPin, LOW); 
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
 Serial.begin(9600);
  BTSerial.begin(9600);
  dht.begin();
}

void loop() {
  /*receive message format 
  [TS=1&GS=1&WS=0&MS=0&FS=0]*/
// send a string to the serial bluetooth device
//?->start of the string ;->end of the string message->Condition&temp*C hudimity%&DAlarm&gas PPM&GAlarm&waterlevel&WAlarm&detected&fire
    output_message="?Condition&temp*C hudimity%&DAlarm&gas PPM&GAlarm&waterlevel&WAlarm&detected&fire;";
    CheckStatus();
    if(counter==0&&GasSystem){
    digitalWrite(GasPower, HIGH);
    }
    MotionDetected();
  if(counter==2){
    Gas_Alarm=GasAlarm();
    digitalWrite(GasPower, LOW);
    DHT_Alarm=DHTAlarm();
    Water_Alarm=WaterAlarm();
    FireDetected();
  }
  else if(counter==5){
    digitalWrite(GasPower, HIGH);
    counter=0;
  }
  Alarm();
  output_message.replace("Condition",(String)Triggered); 
  output_message.replace("hudimity",(String)Humidity_Value);
  output_message.replace("temp", (String)Temp_Value);
  output_message.replace("DAlarm",(String)DHT_Alarm);
  output_message.replace("gas" , (String)Gas_Value);
  output_message.replace("GAlarm",(String)Gas_Alarm);
  output_message.replace("waterlevel" , (String)Water_Level);
  output_message.replace("WAlarm" , (String)Water_Alarm);  
  output_message.replace("detected" , (String)Motion_Detected);
  output_message.replace("fire" , (String)Fire_Detected);
  BTSerial.println(output_message);
  Serial.println(output_message);

 /*for(int i=0;i<15;i++){ 
 LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      
}*/ 

do {
  current_time=millis();
  if(Triggered==1){
         Alarm();
       }
}while(current_time-prev_time<30000);
prev_time=current_time;
 counter++;
}
void CheckStatus(){
  while (BTSerial.available()> 0) {
char data = (char) BTSerial.read();

if(data == '['||detected_start){
  detected_start=true;
  //Serial.println(data);
messageBuffer += data;
if (data == ']'){
input_message = messageBuffer.substring(1, messageBuffer.length() - 1);
messageBuffer = "";
Serial.print(input_message); // send to serial monitor
SetSystems(input_message);
input_message="";
detected_start=false;
counter=0;
break;
}
}
}
}
void SetSystems(String t_message){
  int textCursor=0;
  if(t_message.startsWith("TS=")){
    textCursor=3;
    DHTSystem=(int)t_message[textCursor]-'0';
    //Serial.println(DHTSystem);
    textCursor+=2;
    t_message=t_message.substring(textCursor);
  }
  if(t_message.startsWith("GS=")){
    textCursor=3;
    GasSystem=(int)t_message[textCursor]-'0';
    //Serial.println(GasSystem);
    textCursor+=2;
    t_message=t_message.substring(textCursor);
  }
    if(t_message.startsWith("WS=")){
    textCursor=3;
    WaterSystem=(int)t_message[textCursor]-'0';
    textCursor+=2;
    //Serial.println(WaterSystem);
    t_message=t_message.substring(textCursor);
  }
    if(t_message.startsWith("MS=")){
    textCursor=3;
    MotionSystem=(int)t_message[textCursor]-'0';
    //Serial.println(MotionSystem);
    textCursor+=2;
    t_message=t_message.substring(textCursor);
  }
  if(t_message.startsWith("FS=")){
    textCursor=3;
    FireSystem=(int)t_message[textCursor]-'0';
    //Serial.println(FireSystem);
    textCursor+=2;
    t_message="";
  }
  }
bool DHTAlarm(){
  if(DHTSystem){
    digitalWrite(DHTPower,HIGH);
    delay(100);
    Temp_Value = dht.readTemperature();
    Humidity_Value=dht.readHumidity();
    digitalWrite(DHTPower,LOW);
    if (isnan(Humidity_Value) || isnan(Temp_Value)) {
    //Serial.println("Failed to read from DHT sensor!");
    Temp_Value = -1;
    Humidity_Value=-1;
  }
if(Temp_Value>40||Humidity_Value>80){
  return true;
}
return false;
}
Temp_Value = -1;
Humidity_Value=-1;
return false;
}
bool GasAlarm(){
  if(GasSystem){
  float sensor_volt;
  float RS_gas;
  float ratio;
  float R0 = 0.32;
  int sensorValue = analogRead(Gas_Pin_Analog);
  sensor_volt = ((float)sensorValue / 1024) * 5.0; 
  RS_gas = (5.0 - sensor_volt) / sensor_volt;
  ratio = RS_gas / R0; // ratio = RS/R0 
  Gas_Value = 963.76*pow(ratio, -2.09); 
 if(Gas_Value>200){
 return true;
 }
 return false;
    }
Gas_Value = -1;
return false;
}
bool WaterAlarm() {
  if(WaterSystem){
  digitalWrite(WaterLevelPower, HIGH);  
  delay(10);           
  Water_Level =  analogRead(WaterLevelPin);    
  digitalWrite(WaterLevelPower, LOW);     
if(Water_Level>300){
return true;
}
return false;       
  }
  Water_Level= -1;
return false;
}
void MotionDetected(){
  if(MotionSystem){
    digitalWrite(MotionPower,HIGH);
    delay(20);
    Serial.println(analogRead(DetectMotion));
    if(analogRead(DetectMotion)<300){
  Motion_Detected = 1;
}
else Motion_Detected = 0; 
digitalWrite(MotionPower,LOW);
return;
  }
 Motion_Detected = -1;
}
void FireDetected(){
  if(FireSystem){
    digitalWrite(FirePower,HIGH);
    delay(20); 
      if(digitalRead(FireDigital)==0){
      Fire_Detected = 1;
    }
    else Fire_Detected = 0;  
    digitalWrite(FirePower,LOW);

    return;
  }
  Fire_Detected = -1;
}
void Alarm(){
  if(!(DHT_Alarm)&&!(Gas_Alarm)&&!(Water_Alarm)&&!(Motion_Detected==1)&&!(Fire_Detected==1)){
    noTone(BuzzerPin);
    Triggered=false;
   }
  else if((DHT_Alarm&&DHTSystem)||(Gas_Alarm&&GasSystem)||(Water_Alarm&&WaterSystem)||(Motion_Detected==1)||(Fire_Detected==1&&FireSystem)){
    Triggered=true;
  tone(BuzzerPin, 1200);
  delay(200);
  noTone(BuzzerPin);
  }
}
