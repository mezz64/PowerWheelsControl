#include <Arduino.h>

#define DEBUG

//Defines for ANSI Serial Printouts for DEBUG console
#define ANSI_ESCAPE_SEQUENCE(c) "\33[" c
#define ESC_BOLD_ON ANSI_ESCAPE_SEQUENCE("1m")
#define ESC_BOLD_OFF ANSI_ESCAPE_SEQUENCE("22m")
#define ESC_FG_BLACK ANSI_ESCAPE_SEQUENCE("30m")
#define ESC_FG_RED ANSI_ESCAPE_SEQUENCE("31m")
#define ESC_FG_GREEN ANSI_ESCAPE_SEQUENCE("32m")
#define ESC_FG_YELLOW ANSI_ESCAPE_SEQUENCE("33m")
#define ESC_FG_WHITE ANSI_ESCAPE_SEQUENCE("37m")
#define ESC_BG_BLACK ANSI_ESCAPE_SEQUENCE("40m")
#define ESC_BG_WHITE ANSI_ESCAPE_SEQUENCE("47m")
#define ESC_CURSOR_POS(L,C) ANSI_ESCAPE_SEQUENCE(#L ";" #C "H")
#define ESC_CURSOR_OFF ANSI_ESCAPE_SEQUENCE("?25l")

//DEFINES
const int FORWARD = 1;
const int STOPPED = 0;
const int REVERSE = 2;
const int SHFTFORWARD1 = 1;
const int SHFTFORWARD2 = 2;
const int SHFTREVERSE = 3;
const int DEBUG_INTERVAL = 5000;

// VALUES FOR ACCEL/DECEL/BRAKING RATE
// SET AS NUMBER OF LOOP ITERATIONS (VARIES W/ DEBUG ON/OFF)
const int ACCEL_RATE = 10;
const int DECEL_RATE = 10;
const int BRAKE_RATE = 12;

// VALUES FOR MAX MOTOR SPEED LIMIT
// Mmaximum is 255 for forward directions, and -255 for reverse
// On 20V BATT: 255 - 20V, 155 - 12V, 75 - 6V
const int MAXFWD1 = 125;
const int MAXFWD2 = 200;
const int MAXREV = -100;

//INPUT PINS
const int REMOTE_KILL_PIN = 7;
const int PEDAL_PIN = A3;
const int SHIFT3_PIN = 6;
const int SHIFT2_PIN = 5;
const int SHIFT1_PIN = 4;
const int HEADLIGHT_PIN = 3;
const int PARTY_PIN = 2;

//OUTPUT PINS
const int FWD_PWM = 9;
const int REV_PWM = 10;
const int HEADLIGHT_RELAY = 11;
const int ACC_RELAY = 12;
const int PARTY_RELAY = 8;

// Variables
int currentmode = STOPPED;  // To store current direction of motion
int shiftmode = 0;          // Store shifter position
int pwmspeed = 0;            // To store current commanded speed: value may be from -255 (reverse) to 255 (forward). Zero means stopped

int killState = 0;
int pedalState = 0;
int shift3State = 0;
int shift2State = 0;
int shift1State = 0;
int partyState = 0;
int headlightState = 0;

int okillState = 0;
int opedalState = 0;
int oshift3State = 0;
int oshift2State = 0;
int oshift1State = 0;
int opartyState = 0;
int oheadlightState = 0;

int rawpedal = 0;

unsigned long pedaltime = 0;
unsigned long Dlastrefresh = 0;

// FUNCTION PROTOTYPES
void printStates();
void speedControl();
void forward();
void accel();
void reverse();
void decel();
void slowdown();
void brake();
void commandMotor();
long readVcc();
void sendcursor(int r, int c);
void settext(char* fg, char* bg, char* cup, char* str);
void settextint(char* fg, char* bg, char* cup, int ival);
void settextdoub(char* fg, char* bg, char* cup, float ival);

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); //PIN 13 on ARDUINO PRO MINI
  pinMode(REMOTE_KILL_PIN, INPUT_PULLUP);
  pinMode(PEDAL_PIN, INPUT);
  pinMode(SHIFT3_PIN, INPUT_PULLUP);
  pinMode(SHIFT2_PIN, INPUT_PULLUP);
  pinMode(SHIFT1_PIN, INPUT_PULLUP);
  pinMode(HEADLIGHT_PIN, INPUT_PULLUP);
  pinMode(PARTY_PIN, INPUT_PULLUP);

  pinMode(FWD_PWM, OUTPUT);
  pinMode(REV_PWM, OUTPUT);
  pinMode(ACC_RELAY, OUTPUT);
  digitalWrite(ACC_RELAY, HIGH);  //Relay Board is active on low, start off
  pinMode(PARTY_RELAY, OUTPUT);
  digitalWrite(PARTY_RELAY, HIGH);  //Relay Board is active on low, start off
  pinMode(HEADLIGHT_RELAY, OUTPUT);
  digitalWrite(HEADLIGHT_RELAY, HIGH);  //Relay Board is active on low, start off

  /************** SET PWM frequency. default is 450hz  **********************/
  TCCR2B = TCCR2B & B11111000 | B00000010; //4 kHz pins 9&10, timer1
  //TCCR2B = TCCR2B & B11111000 | B00000001; //32 kHz uno

  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print(ESC_CURSOR_OFF);
    settext(ESC_FG_BLACK, ESC_BG_WHITE, ESC_CURSOR_POS(1, 1), "VEHICLE DEBUG INTERFACE");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(2, 1), "PEDAL_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(3, 1), "PWM_OUTPUT: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(4, 1), "DRIVE_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(6, 1), "KILL_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(8, 1), "SHFT3_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(9, 1), "SHFT2_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(10, 1), "SHFT1_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(11, 1), "HEADLIGHT_STATE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(12, 1), "PARTY_STATE: ");

    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(14, 1), "SHIFT_MODE: ");
    settext(ESC_FG_WHITE, ESC_BG_BLACK, ESC_CURSOR_POS(15, 1), "RAW_PEDAL: ");
  #endif
}

void loop() {
  // Sample input pins
  okillState = killState;
  killState = !digitalRead(REMOTE_KILL_PIN);  //Ground switched so invert to make 1=ON
  oshift3State = shift3State;
  shift3State = !digitalRead(SHIFT3_PIN);  //Ground switched so invert to make 1=ON
  oshift2State = shift2State;
  shift2State = !digitalRead(SHIFT2_PIN);  //Ground switched so invert to make 1=ON
  oshift1State = shift1State;
  shift1State = !digitalRead(SHIFT1_PIN);  //Ground switched so invert to make 1=ON
  opartyState = partyState;
  partyState = !digitalRead(PARTY_PIN);  //Ground switched so invert to make 1=ON
  oheadlightState = headlightState;
  headlightState = !digitalRead(HEADLIGHT_PIN);  //Ground switched so invert to make 1=ON

  // Read pedal state, pedal range of 0.8V (164) to 4.2V (860) ---750
  opedalState = pedalState;  //Store previous 
  pedalState = analogRead(PEDAL_PIN);
  rawpedal = pedalState;
  if (pedalState <= 185) { // pedal rests at 175ish from the ADC, make sure we don't move
    pedalState = 0; 
  } else {
    pedalState = map(pedalState, 185, 735, 0, 1023); //Remap usable range
  }  

  // Reverse - p1 & p3
  // Forward1 - p3
  // Forward2 - p2
  if (shift1State == HIGH && shift2State == LOW && shift3State == HIGH) { shiftmode = SHFTREVERSE; }
  else if (shift1State == LOW && shift2State == LOW && shift3State == HIGH) { shiftmode = SHFTFORWARD1; }
  else if (shift1State == LOW && shift2State == HIGH && shift3State == LOW) { shiftmode = SHFTFORWARD2; }
  else { shiftmode = 0; }

  digitalWrite(ACC_RELAY, LOW); // Turn on ACC

  if (millis() > 3000 && killState == LOW) //Give things time to settle on start, don't move if kill is active.
  {
    switch (shiftmode) 
    {
      case SHFTFORWARD1: 
        pedalState = map(pedalState, 0, 1023, 0, MAXFWD1); // Adjust pedal input for speed range
        if (pedalState > MAXFWD1) { pedalState = MAXFWD1; } //Bounds Check
        break;
      case SHFTFORWARD2:
        pedalState = map(pedalState, 0, 1023, 0, MAXFWD2); // Adjust pedal input for speed range
        if (pedalState > MAXFWD2) { pedalState = MAXFWD2; } //Bounds Check
        break;
      case SHFTREVERSE:
        pedalState = map(pedalState, 0, 1023, 0, MAXREV); // Adjust pedal input for speed range
        if (pedalState < MAXREV) { pedalState = MAXREV; } //Bounds Check
        break;
      default:
        pedalState = 0;
        break;
    }
    speedControl();  // Parse pedalState to determine accel/decel
  } 
  else
  {
    brake(); // Bring things to a stop
  }

  // Handle hedlight and party light switch state changes
  if (headlightState == HIGH) {
    digitalWrite(HEADLIGHT_RELAY, LOW);
  } else {
    digitalWrite(HEADLIGHT_RELAY, HIGH);
  }
  if (partyState == HIGH) {
    digitalWrite(PARTY_RELAY, LOW);
  } else {
    digitalWrite(PARTY_RELAY, HIGH);
  }

  #ifdef DEBUG
    settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(2, 16), pedalState);
    settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(3, 16), pwmspeed);
    settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(4, 16), currentmode);

    if (killState != okillState) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(6, 20), killState); }
    if (shift3State != oshift3State) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(8, 15), shift3State); }
    if (shift2State != oshift2State) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(9, 15), shift2State); }
    if (shift1State != oshift1State) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(10, 15), shift1State); }
    if (headlightState != oheadlightState) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(11, 20), headlightState); }
    if (partyState != opartyState) { settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(12, 15), partyState); }

    settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(14, 15), shiftmode);
    settextint(ESC_FG_RED, ESC_BG_BLACK, ESC_CURSOR_POS(15, 13), rawpedal);
  #endif
}

void speedControl()  //Parse pedal input
{
  //Setup general bounding conditions to compare pedal input against output
  if (pedalState > pwmspeed)
  {
    if (currentmode == FORWARD || currentmode == STOPPED)
    {
      // We want to go faster
      accel();
    }
    else if (currentmode == REVERSE)
    {
      // We want to go slower
      accel();
    }
  }
  else if (pedalState < pwmspeed)
  {
    if (currentmode == REVERSE || currentmode == STOPPED)
    {
      // We want to go faster
      decel();
    }
    else if (currentmode == FORWARD)
    {
      // We want to go slower
      decel();
    }
  }
  else if (pedalState == pwmspeed)
  {
    // All is good
  }

  //Update drive mode
  if (pwmspeed > 0)
  {
    currentmode = FORWARD;
  } else if (pwmspeed < 0) {
    currentmode = REVERSE;
  } else if (pwmspeed == 0) {
    currentmode = STOPPED;
  }
}

void accel()
{
  int call_Rate = pedalState - pwmspeed;
  if ( abs(call_Rate) <= ACCEL_RATE ){
    pwmspeed = pedalState;
  } else {
    pwmspeed += ACCEL_RATE;
  }
  commandMotor();
}

void decel()
{
  int call_Rate = pedalState - pwmspeed;
  if ( abs(call_Rate) <= DECEL_RATE ){
    pwmspeed = pedalState;
  } else {
    pwmspeed -= DECEL_RATE;
  }
  commandMotor();
}

void brake()
{
  if(pwmspeed>0)  // slow from forward direction
  {
    pwmspeed-=BRAKE_RATE;
    if(pwmspeed<0)
    {
      pwmspeed=0;
    }
  } 
  else if(pwmspeed<0)  // slow from reverse direction
  {
    pwmspeed+=BRAKE_RATE;
    if(pwmspeed>0)
    {
      pwmspeed=0;
    }
  }
  commandMotor();
}


void commandMotor()
{
   /* Bounds check, if greater than max, set it to 255 max. 
   * This is need if the accel/decel rate is not a perfect multiple of the max speed(s)
   * Good safety measure anyways since the PWM is limited to a byte value of 255 max
   * yet the pwm is set to an int. 
   * Bizarre (aka unsafe) stuff will happen if you try to set the PWM value > 255. 
   */
  if (pwmspeed > 254) //bounds check, if greater than max, set it to max. This is need if the accel/decel rate is not a perfect multiple of the max speed(s)
  {
    pwmspeed=255;
  }
  
  if (pwmspeed < -254) // same deal, except negative.
  {
    pwmspeed = -255;
  }
  
  // send the command to the motor
  if(pwmspeed==0)
  { // All stopped
    analogWrite(FWD_PWM,0);
    analogWrite(REV_PWM,0);
    currentmode=STOPPED;
  } 
  else if(pwmspeed>0)
  { // forward motion
    analogWrite(REV_PWM,0);
    analogWrite(FWD_PWM,pwmspeed);
  } 
  else 
  { // reverse motion
    analogWrite(FWD_PWM,0);
    analogWrite(REV_PWM,-1*pwmspeed); 
  }
  delay(50); 
}

// Read internal voltage reference
// Not currently used, but may be suitable to calibrate PWM feedback in the future
long readVcc() { 
  long result; // Read 1.1V reference against AVcc 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); // Wait for Vref to settle 
  ADCSRA |= _BV(ADSC); // Convert 
  while (bit_is_set(ADCSRA,ADSC)); 
  result = ADCL; 
  result |= ADCH<<8;
  //Original -  1126400L
  //China cable - 1109248L
  result = 1109248L / result; // Back-calculate AVcc in mV 
  return result;
}

void sendcursor(int r, int c)
{ // set ANSI terminal cursor to specified row and column
  char str[12];
  sprintf(str,"\33[%d;%dH",r,c);
  Serial.print(str);
}

void settext(char* fg, char* bg, char* cup, char* str)
//void settext(char* fg, char* bg, char* cup, String str)
// send text to terminal with foreground color, background color, cursor position
{
  Serial.print(cup); 
  Serial.print(fg);
  Serial.print(bg);
  Serial.print(str);
}

void settextint(char* fg, char* bg, char* cup, int ival)
// send text to terminal with foreground color, background color, cursor position
{
  Serial.print(cup);
  Serial.print("                 ");
  Serial.print(cup); 
  Serial.print(fg);
  Serial.print(bg);
  Serial.print(ESC_BOLD_ON);
  char sbuffer[6];
  if (ival > 2) {
    snprintf(sbuffer, sizeof(sbuffer), "+%03d", ival);
  } else if (ival < 0) {
    snprintf(sbuffer, sizeof(sbuffer), "-%03d", abs(ival));
  }else { //binary only
    snprintf(sbuffer, sizeof(sbuffer), "%01d", ival);
  }
  
  Serial.print(sbuffer);
  Serial.print(ESC_BOLD_OFF);
}

void settextdoub(char* fg, char* bg, char* cup, float ival)
// send text to terminal with foreground color, background color, cursor position
{
  Serial.print(cup);
  Serial.print("                 ");
  Serial.print(cup); 
  Serial.print(fg);
  Serial.print(bg);
  Serial.print(ESC_BOLD_ON);
  // char sbuffer[10];
  // sprintf(sbuffer, dtostrf( ival, 6, 2, "%f" ));
  // Serial.print(sbuffer);
  Serial.print(ival);
  Serial.print(" A");
  Serial.print(ESC_BOLD_OFF);
}
