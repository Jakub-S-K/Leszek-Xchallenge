#define sharp_RF_Pin 10 // PA0 ESP 15
#define sharp_LF_Pin 11 // PA1 ESP 34
#define sharp_RS_Pin 12 // PA2
#define sharp_LS_Pin 13 // PA3

#define ground_RF_Pin 30 //PA9
#define ground_LF_Pin 32 //PA11
#define ground_RB_Pin 29 //PA8
#define ground_LB_Pin 31 //PA10

//ENGINE PINOUT
//ENGINE L albo prawy 
#define EngineL_PWM 21 //PWM pb10
#define EngineL_F 20 //Przut gdy wysoki  :3 pb2
#define EngineL_B 19 //tył gdy wysoki  pb1
//-----------------------
//GDY ENGINE F i B są wysokie jednocześnie to jest zwarcie
//-----------------------
//ENGINE R albo lewy
#define EngineR_PWM 18 //PWM pb0
#define EngineR_F 17 //przut :3 pa7
#define EngineR_B 16 //tył pa6

#define START_PIN 40 //PB4 CHYBA!

#define DEBUG
bool debug = true;

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
    delay(100);
    Serial.begin(115200);  
  #endif
  
  //Config
  analogReadResolution(10);
  
  delay(10);
  
}



//engines
bool EngineL = false; // false backward, true forward
bool EngineR = false; // false backward, true forward
unsigned short EngineLPWM = 0; 
unsigned short EngineRPWM = 0;

//SENSORS

//ground sensor L=Left R=Right F=Front B=Back
bool ground_LF = false;
bool ground_RF = false;
bool ground_LB = false;
bool ground_RB = false;

//FRONT SHARP
short sharp_LF = 0;
short sharp_RF = 0;
//SIDE SHARP
short sharp_LS = 0;
short sharp_RS = 0;




// TEMP
short tmpsharp_LF = 0;
short tmpsharp_RF = 0;

void Move(short speed, float speedProportion, bool LE_direction, bool RE_direction){ //s
  //speed 
  
  //bananas rotate ex 90deg clockwise -90deg counterclockwise 
}
void MoveAleInaczej(short x, short y){
  //niewiem
}

void readsensor(int numL,int numR)  { 

  tmpsharp_LF = analogRead(numL);
  tmpsharp_RF = analogRead(numR);
}


void loop() {
  // put your main code here, to run repeatedly:
  while(!digitalRead(START_PIN));
  readsensor(34,15);
  sharp_LF = tmpsharp_LF;
  sharp_RF = tmpsharp_RF;
  if(debug){
    Serial.print(sharp_LF);
    Serial.print(',');
    Serial.print(sharp_RF);
    Serial.print(',');
    Serial.print(sharp_RF + sharp_LF);
    Serial.print(',');
    Serial.println(sharp_LF - sharp_RF);  
  }
  
}