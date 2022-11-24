#define sharp_RF_Pin 10 // PA0 ESP 15
#define sharp_LF_Pin 11 // PA1 ESP 34
#define sharp_RS_Pin 12 // PA2
#define sharp_LS_Pin 13 // PA3

#define ground_RF_Pin 30 // PA9
#define ground_LF_Pin 32 // PA11
#define ground_RB_Pin 29 // PA8
#define ground_LB_Pin 31 // PA10

// ENGINE PINOUT
// ENGINE L albo prawy
#define EngineL_PWM 21 // PWM pb10
#define EngineL_F 20   // Przut gdy wysoki  :3 pb2
#define EngineL_B 19   // tył gdy wysoki  pb1
//----------------------t -
// GDY ENGINE F i B są wysokie jednocześnie to jest zwarcie
//-----------------------
// ENGINE R albo lewy
#define EngineR_PWM 18 // PWM pb0
#define EngineR_F 17   // przut :3 pa7
#define EngineR_B 16   // tył pa6

#define ENEMY_TRESHOLD 300

#define LEFT_FORWARD (1)
#define LEFT_BACKWARD (1 << 1)
#define RIGHT_FORWARD (1 << 2)
#define RIGHT_BACKWARD (1 << 3)
#define DIR_STOP (1 << 4)

#define START_PIN 40 // PB4
#define STOP_PIN 41  // PB5

enum MODE
{
  BORDER,
  SCAN,
  PUSH
} logic = MODE::SCAN;

// engines
bool EngineL = false; // false backward, true forward
bool EngineR = false; // false backward, true forward
unsigned short EngineLPWM = 0;
unsigned short EngineRPWM = 0;

// SENSORS

// ground sensor L=Left R=Right F=Front B=Back
bool ground_LF = false;
bool ground_RF = false;
bool ground_LB = false;
bool ground_RB = false;

bool programStart = true;

// FRONT SHARP
short sharp_LF = 0;
short sharp_RF = 0;
// SIDE SHARP
short sharp_LS = 0;
short sharp_RS = 0;

inline void Move(uint8_t throttle, uint32_t directions);
inline void MoveAleInaczej(short x, short y);
inline void readSensors();
bool setState();

void setup()
{

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

void loop()
{

  if (programStart)
  {
    programStart = !digitalRead(START_PIN);
    return;
  }
  if (digitalRead(STOP_PIN)) // TODO: fix condition statement
  {
    while (true)
      ;
  }

  readSensors();

  if (setState())
  { // setState return true on abrupt change needing action immediately
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

inline void readSensors()
{
  sharp_LF = analogRead(sharp_LF_Pin);
  sharp_RF = analogRead(sharp_RF_Pin);
  sharp_LS = analogRead(sharp_LS_Pin);
  sharp_RS = analogRead(sharp_RS_Pin);
}

inline void Move(uint8_t throttle, uint32_t directions)
{
  if (directions & DIR_STOP)
  {
    digitalWrite(EngineL_B, LOW);
    digitalWrite(EngineL_F, LOW);
    digitalWrite(EngineR_B, LOW);
    digitalWrite(EngineR_F, LOW);
    return;
  }
  if (directions & LEFT_FORWARD)
  {
    digitalWrite(EngineL_B, LOW);
    digitalWrite(EngineL_F, HIGH);
  }
  else if (directions & LEFT_BACKWARD)
  {
    digitalWrite(EngineL_F, LOW);
    digitalWrite(EngineL_B, HIGH);
  }

  if (directions & RIGHT_FORWARD)
  {
    digitalWrite(EngineR_B, LOW);
    digitalWrite(EngineR_F, HIGH);
  }
  else if (directions & RIGHT_BACKWARD)
  {
    digitalWrite(EngineR_F, LOW);
    digitalWrite(EngineR_B, HIGH);
  }
}

bool setState()
{ // Set state machine based on sensor input
  switch (logic)
  {
  case MODE::BORDER:
    /* code */
    break;

  default:
    break;
  }

  return true;
}
