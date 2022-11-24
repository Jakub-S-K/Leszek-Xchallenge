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
//----------------------t -
//GDY ENGINE F i B są wysokie jednocześnie to jest zwarcie
//-----------------------
//ENGINE R albo lewy
#define EngineR_PWM 18 //PWM pb0
#define EngineR_F 17 //przut :3 pa7
#define EngineR_B 16 //tył pa6

#define START_PIN 40 //PB4 CHYBA!

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

inline void Move(short speed, float speedProportion, bool LE_direction, bool RE_direction);
inline void MoveAleInaczej(short x, short y);
inline void readSensors();
bool setState();

void setup() {

  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(sharp_LF_Pin, INPUT);
  pinMode(sharp_RF_Pin, INPUT);
  pinMode(sharp_LS_Pin, INPUT);
  pinMode(sharp_RS_Pin, INPUT);

#ifdef DEBUG
  delay(100);
  Serial.begin(115200);  
#endif
}

void loop() {

  while(digitalRead(START_PIN)); //Wait for LOW voltage to start
  readSensors();
  if(setState()) { //setState return true on abrupt change needing action immediately
    return; 
  }

#ifdef DEBUG
  Serial.print(sharp_LF);
  Serial.print(',');
  Serial.print(sharp_RF);
  Serial.print(',');
  Serial.print(sharp_RF + sharp_LF);
  Serial.print(',');
  Serial.println(sharp_LF - sharp_RF);  
#endif
  
}

inline void readSensors()  { 
  sharp_LF = analogRead(sharp_LF_Pin);
  sharp_RF = analogRead(sharp_RF_Pin);
  sharp_LS = analogRead(sharp_LS_Pin);
  sharp_RS = analogRead(sharp_RS_Pin);
}

void Move(short speed, float speedProportion, bool LE_direction, bool RE_direction){ //s
  //speed 
  //bananas rotate ex 90deg clockwise -90deg counterclockwise 
}

void MoveAleInaczej(short x, short y){
  //niewiem
}

bool setState() { // Set state machine based on sensor input

  return true;
}
