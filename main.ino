#include <RBDdimmer.h>//
//#define USE_SERIAL  Serial
#define outputPin  3 //PWM OUT DIMMER
#define zerocross  2 // ZERO CROSS SIGNAL
//dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero
#include <Adafruit_MAX31865.h>
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      429.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
#include<Nextion.h>
//needs modification to work with serial protocol

//////DISPLAY OUTPUT///////
NexText temp=NexText(0,3,"t1");                   /// TEMPERATURE: OUTPUT
NexText perc=NexText(0,5,"t2");                   /// PERCENTAGE OF POWER: PWM/255
//////DISPLAY INPUT///////
NexSlider slider=NexSlider(0,4,"h0");          /// SLIDER NOT USED IN PID
NexNumber temp_set = NexNumber(0,12,"temp_set");  /// SET TEMPERATURE: INPUT
NexDSButton mode_set = NexDSButton(0, 1, "mode");  /// MODE BUTTON: DUAL STATE BUTTON NOT USED IN PID
//////DISPLAY OBJECTS TO LISTEN/////
NexTouch *nex_list[]={
 &slider,&mode_set,
 &temp_set,
 NULL
};
//////VARIABLE DEFINITION///////
uint32_t mode_set_in=0;            ///INITIAL STATE MODE: 0=MANUAL, 1=AUTO (PID)
uint32_t temp_set_in=0;         ///TEMPERATURE SET AS VARIABLE
int outPerc = 0;                ///PERCENTAGE OF POWER AS VARIABLE     
//////PID PARAM - ADJUST TO MODIFY THE RESPONSE//////
  float kp=15;
  float ki=10;
///////SETUP - NEEDED///////
void setup() {
  Serial.begin(9600);
  nexInit();
  max.begin(MAX31865_2WIRE);
  dimmer.begin(NORMAL_MODE, ON);                    ///dimmer initialisation: name.begin(MODE, STATE) 
  slider.attachPop(manual_set_evh,&slider);          ///FUNCTION TO LISTEN SLIDER
  mode_set.attachPop(mode_evh,&mode_set);                   ///FUNCTION TO LISTEN MODE
  temp_set.attachPop(temp_set_evh,&temp_set);       ///FUNCTION TO LISTEN TEMPERATURE SET
}
////////OUTPUT FUNCTIONS - UPDATE ///////////
///UPDATE TEMPERATURE
void updateTemp(){
  float t=max.temperature(RNOMINAL, RREF);
  char buff_temp[6];
  dtostrf(t, 4, 2, buff_temp);
  temp.setText(buff_temp);
  }
///UPDATE PERCENTAGE TO DISPLAY
void updatePerc(int p){
  char buff_perc[6];
  dtostrf(p, 4, 2, buff_perc);
  perc.setText(buff_perc);
  }

///MODE MANUAL/AUTO READING
int mode_evh(){
  mode_set.getValue(&mode_set_in);
  return mode_set_in;
}
/// MANUAL SETUP BY SLIDER
void manual_set_evh(){
  long n=0;
  slider.getValue(&n);
  int outPerc=(int)n;
  dimmer.setPower(outPerc); // name.setPower(0%-100%)
  updatePerc(outPerc);
  updateTemp();
}
/// AUTO SETUP BY PID
void temp_set_evh(){
  temp_set.getValue(&temp_set_in);
  float SETPOINT = temp_set_in;
  float v=max.temperature(RNOMINAL, RREF);
  float e=SETPOINT-v;
  int out = (int) (prop(e) + intgr(e));
  out = map(out,0,500,0,255);
  out =  constrain(out,0,100);
  int outPerc=(int)out;
  dimmer.setPower(outPerc); // name.setPower(0%-100%)
  updatePerc(outPerc);
  updateTemp();
}
///VARIABLE DECLARATION - DELAY() DO NOT WORK WITH NEXTION
unsigned long t1, dt;
///////SETUP - NEEDED///////
void loop() {
  nexLoop(nex_list);
  dt=millis()-t1;
  if(dt>100){
  mode_evh();
  switch(mode_set_in){
    case 0:
          manual_set_evh();
    break;
    case 1:
          temp_set_evh();
    break;
    }
    t1=millis();
  }
}
//////PID FUNCTIONS//////
///PROPORTIONAL
float prop(float e){
  float p=e*kp;
  return p;
  }
///INTEGRATIVE (AVG)
#define blen 10
float buf[blen];
float intgr(float e){
  buf[0]=e;
  for(int j=blen-1;j>0;j--){
    buf[j]=buf[j-1];
  }
  float m=0;
  for(int j=0;j<blen;j++){
    m+=buf[j];
  }
  m=m/(float)blen;
  float i=m*ki;
  return i;
}