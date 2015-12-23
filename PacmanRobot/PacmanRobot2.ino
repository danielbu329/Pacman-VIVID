#define DEBUG_RF  0 // Debug messages related to updating the game state + internals from RF
#define PRINT_DECISION // Debug messages used during decide direction

//TODO What are these? To store a color? Place in GLobal Variable list?
uint8_t red = 0;
uint8_t blue = 0;
uint8_t green = 0;


#define USE_RADIO
//#define MATLAB  //TODO When uncomment
  
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <TimerOne.h>
//#include <TimerThree.h>
#include <SPI.h>
#include <AccelStepper.h>
#include "RF24.h"
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include "LineSensor.h"
#include "Game.h"
#include "Map.h"

/* note: data types and variables have been grouped together for ease of reading.
   This leads to inefficient allocation of memory space, but for now we have plenty.
   Once program is working, suggest moving relevant sections into different .h and .c
   files, as well as converting as much code as possible to c++ rather than c.
   
   For now, don't move around things.
*/

/*
 * Radio definitions
 */

#define BROADCAST_CHANNEL 72
static const uint64_t pipe = 0xF0F0F0F0E1LL;

/*
 *    Hardware definitions 
 *    Robot Colors, Motors, LineSensors, Radio (TODO WiFi)
 */

 //TODO Understanding:
 // LED_STRIP_PIN(43) is the pin for the STRIP of all 20 Neo Pixels on the Robot
 // Each is then numbered into pacStrip, based on how they are placed.

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define LED_STRIP_PIN 43 //TODO What strip is this?
// eyes connected to pins 42,44,46,48
#define EYE_LL  42
#define EYE_LR  44
#define EYE_RL  46
#define EYE_RR  48

#define NUMPIXELS      20 //Number of NeoPixels Attached to the Arduino
 // TODO  Regions ?                  top,        right,         bottom,       left   
const uint8_t pacStrip [4][5] = {{0,1,8,9,10},{2,3,11,12,13},{4,5,14,15,16},{6,7,17,18,19}};

uint8_t rgb[3]; //Stores the Colours of the Current Robot

//Uses NeoPixel Constuctor. (n number leds, pin number, led type)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

//AccelStepper (4 Wire Half Step Motor(HALF4WIRE),pins 1-4,don't autoenable OUTPUT mode for pins)
//(Pins 22-29, 32-39)
AccelStepper m_topLeft(AccelStepper::HALF4WIRE,26,28,22,24,false);
AccelStepper m_topRight(AccelStepper::HALF4WIRE,27,29,23,25,false);
AccelStepper m_bottomLeft(AccelStepper::HALF4WIRE,36,38,32,34,false);
AccelStepper m_bottomRight(AccelStepper::HALF4WIRE,37,39,33,35,false);

// LineSensor (Pins 0 to 15)
LineSensor lineTop(3,2,1,0);
LineSensor lineLeft(7,6,5,4);
LineSensor lineBottom(11,10,9,8);
LineSensor lineRight(15,14,13,12);

//Radio (Pins 49,53)
RF24 radio(49,53);

// TODO to be inserted: button(s) (x1 or x2) for mode select
// TODO Change to WiFi

/*
 * Local definitions
 */
typedef enum {PACMAN=0, GHOST1, GHOST2, GHOST3} playerType_t;


#define M_SPEED 225
#define M_SLOW  150
#define STEPPER_MAX_SPEED 1000
#define MAX_ALIGN_TIME 1000

/*
 * Global Variables
 * TODO Explanations for each variable
 */

volatile int       moveFlag = 0;
playerType_t       currPlayer;
robot_t            *thisRobot;
position_t         goal;
heading_t          globalHeading;
volatile heading_t currentHeading;
unsigned long      aiTime = 0;
unsigned long      loopTime = 0;
unsigned long      lastIntersection = 0;
bool               animationToggle = 0;
bool               readingRadio = 0;
bool               radio_flag = 0;
game_state_t       game;

// Extra space to prevent corruption of data, due to the required 32 byte payload.
uint8_t __space[21];

#ifdef MATLAB
  // Initialise the robots as stopped
  int curr_command = STOP;
#else
  // Initialise robots as NOP (normal)
  int curr_command = NOP;
#endif

// Updated by MANUAL_OVERRIDE
uint8_t       player_direction = 0;
unsigned long manual_override_timer = 0;
unsigned long lastRadioUpdateTime = 0;



/*
 * Functions Definitions
 */

/*
 * Setup Functions: 
 */
void InitRadio(void);
void InitMotors(void);
void InitLightSensors(void);
void CalibrateLightSensors(void);
void InitLeds(void);
void SetCurrPlayerLeds();

void InitHeading(void); //Used in both Setup and Loop

/*
 * Loop Functions: 
 */

void GhostAnimation(void); //Ghost Animations, Eyes direction
void PacmanAnimation(void); 
//Radio
void UpdateGame(void); //Uses radio
void SetChecksum(void); //Used by UpdateGame, sets checksum of game variable locally
uint8_t GetChecksum(game_state_t *g); //Gets Checksum of a game_state_t
//Movement
void MoveRobot(void);
void MoveFlag(void); //TODO Not used?
void UpdateSpeed(AccelStepper *thisMotor,int newSpeed);
void GetMotorAlignment(AccelStepper **m_fL,AccelStepper **m_fR,AccelStepper **m_bL,AccelStepper **m_bR);
void GetLineAlignment(LineSensor **l_F,LineSensor **l_B);
void LineFollow(void);
bool AlignToIntersection(void);
uint8_t DetectIntersection(void);
void DecideDirection(uint8_t options);
//Map Navigation
void SetGoal(void); //TODO Not used? For AI later?
uint8_t ExpandMap(void);
bool IsIntersection(int x, int y);
bool IsOpen(int x, int y, heading_t dir);
bool CollisionDetect(heading_t h);
uint8_t* CheckSquare(int x, int y); //SILLY that it has to be uint8_t*! BUT ARDUINO WON'T COMPILE OTHERWISE!!
uint8_t* Expand(int *x,int *y, heading_t h);
uint8_t* ExpandSingle(int *x,int *y,heading_t h);
void CalcNewSquare(int *x,int*y,heading_t h,int d);
uint8_t HeadingToBinary(heading_t h);
char BinaryToHeading(uint8_t h);

/*
 * Other Functions
 */
//Debug
void PrintGame(void);
void DebugColor(); 
void enter_robot_location(robot_t * entry);
//TODO Not Used?
void init_game(void); 
void init_game_map(void); 



/*
 *
 * SETUP FUNCTIONS 
 *
 */

/*
 * SETUP FUNCTION FOR ARDUINO
 */
void setup() {
    // put your setup code here, to run once:
    Serial.begin(57600);
    globalHeading = '0';
    currentHeading = '0';
    InitLeds();
    //while(!Serial.available()){}
    InitMotors();
    InitLightSensors();
    randomSeed(micros()); // Initialised Psuedo Random Number Generator
    #ifdef USE_RADIO
      InitRadio();
    #endif

    // purple         TODO ?
    red = 50; green = 0; blue = 50;
    DebugColor();
    pinMode(47, OUTPUT);
    digitalWrite(47,LOW);
    pinMode(45,INPUT_PULLUP);
    // Wait for first button press
    while(digitalRead(45)){
    }
    delay(50);
    while(!digitalRead(45));
    delay(50);

    //TODO This is to setup the current Robot? Select from the 4 Types: Pacman, 
    bool longTouch = false;
    int switchPlayer = 0; // starts with PACMAN
    unsigned long buttonTime;
    while(longTouch==false){ 
        currPlayer = (playerType_t)switchPlayer;
        SetCurrPlayerLeds();
        while (digitalRead(45));
        delay(50);
        buttonTime = millis();
        while (!digitalRead(45)){
            if(millis()-buttonTime>2000){
                longTouch = true;
                break;
            }
        }
        if(longTouch==false){
            switchPlayer = (switchPlayer+1)%4;
        }
    }
    
    //TODO What is happening
    // software debounce with this delay
    // blue
    red = 0; green = 50; blue = 0;
    DebugColor(); 
    while(!digitalRead(45));
    delay(50);
    // Wait for second button press and continually calibrate
    while (digitalRead(45)) {
        CalibrateLightSensors();
    }
    delay(50);

    while (!digitalRead(45));
    InitHeading();
    
    #ifdef USE_RADIO
        // Init pacman waiting for input, will time-out after 10 seconds
        if (currPlayer == PACMAN) {
            curr_command = MANUAL_OVERRIDE;
            manual_override_timer = millis();
            globalHeading = '0';  
        }
    #endif
    SetCurrPlayerLeds(); 
    game.command = 0;
}

void InitLeds(){
    pixels.begin();
    pinMode(EYE_LL,OUTPUT);
    pinMode(EYE_LR,OUTPUT);
    pinMode(EYE_RL,OUTPUT);
    pinMode(EYE_RR,OUTPUT);
    int i;
    for(i=0;i<NUMPIXELS;i++){
       pixels.setPixelColor(i, pixels.Color(0,0,0));
    }
    pixels.show();
}

void InitMotors() {
    Timer1.detachInterrupt();
    m_topLeft.setMaxSpeed(STEPPER_MAX_SPEED);
    m_topRight.setMaxSpeed(STEPPER_MAX_SPEED);
    m_bottomLeft.setMaxSpeed(STEPPER_MAX_SPEED);
    m_bottomRight.setMaxSpeed(STEPPER_MAX_SPEED);
    m_topLeft.setSpeed(0);
    m_topRight.setSpeed(0);
    m_bottomLeft.setSpeed(0);
    m_bottomRight.setSpeed(0);
    m_topLeft.disableOutputs();
    m_topRight.disableOutputs();
    m_bottomLeft.disableOutputs();
    m_bottomRight.disableOutputs();
    Timer1.initialize(500);  // 100 us hopefully fast enough for variable speeds
    Timer1.attachInterrupt( MoveRobot );  // interrupt sets motion flag
}

void InitLightSensors(){
  lineTop.init_calibrate();
  lineRight.init_calibrate();
  lineBottom.init_calibrate();
  lineLeft.init_calibrate();
}

void InitRadio() {
  radio.begin();
  radio.openReadingPipe(0,pipe);
  radio.setChannel(BROADCAST_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.startListening();
  radio.setPayloadSize(32);
  attachInterrupt(3, check_radio, FALLING); //When something receives something from pin 3....?
}

void check_radio() {
   Serial.println("check_radio...");
   radio_flag = 1; // global var
   Serial.println("check_radio - done");
}

void CalibrateLightSensors() {
  lineTop.calibrate();
  lineRight.calibrate();
  lineBottom.calibrate();
  lineLeft.calibrate();
}

void SetCurrPlayerLeds(){
  switch (currPlayer) {
    case PACMAN :
      rgb[0] = 255;
      rgb[1] = 255;
      rgb[2] = 0;
      break;
    case GHOST1 : // Blinky (RED)
      rgb[0] = 255;
      rgb[1] = 0;
      rgb[2] = 0;
      break;
    case GHOST2 : // Inky (BLUE) TODO More of a Cyan Color?
      rgb[0] = 2;
      rgb[1] = 2;
      rgb[2] = 225;
      break;
    case GHOST3 : // TODO What color is it? EDIT: Pinky or Clyde
      rgb[0] = 255;
      rgb[1] = 184;
      rgb[2] = 255;
      break;
    default :
      return;
      break;
  }

  int i;
  for (i = 0; i < NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(rgb[0],rgb[1],rgb[2]));
  }
  pixels.show();
}
  
// TODO NOT USED? REMOVE?
// Initialises the game and waits for the host to send start command
void init_game() {
  // To be replaced with LCD display and button toggling!
  switch(currPlayer){
    case PACMAN : thisRobot = &game.pac; break;    
    default     : thisRobot = &game.g[currPlayer-1]; break;
  }
}



/*  
 * 
 * MAIN FUNCTIONS 
 *
 */

// A simple count to "hold" a person's input
// NOTE: This simply prevents the return from manual control to AI driven for a period of time, the
//       game is still updated normally, including if the person changes their input
#define MANUAL_OVERRIDE_THRESHOLD 10000

 /*
  * LOOP FUNCTION FOR ARDUINO
  */
void loop() {
    //Serial.println("start loop");
    //Serial.print("curr_command: ");
    //Serial.println(curr_command);
    //Serial.print("globalHeading: ");
    //Serial.println(globalHeading);
    delay(10);
    //  if (!digitalRead(45)) {
    //    delay(50);
    //    while (!digitalRead(45));
    //    delay(50);
    //    setup();
    //  }
    #ifdef USE_RADIO
        //Serial.println(millis() - manual_override_timer);  
        if (radio_flag) {
            UpdateGame();
        }
        // Whilst the last manual_override is in effect, don't update the current command
        //    if (false) {
        if ((curr_command == MANUAL_OVERRIDE) && ((millis() - manual_override_timer) >= MANUAL_OVERRIDE_THRESHOLD)) {
            // Once threshold reached, return to standard play (w/ AI)
            manual_override_timer = 0;
            curr_command = NOP;
            if (DEBUG_RF){Serial.println("RF: MANUAL_OVERRIDE PERIOD LAPSED : RETURN TO RANDOM PLAY");}
            InitHeading();
            Serial.print("Found heading: '");
            Serial.print(globalHeading);
            Serial.println("'");
            rgb[0] = 220;
            rgb[1] = 220;
            rgb[2] = 0;
        }
        // Make sure the game header is consistent with the checksum 
        if (game.header == GetChecksum(&game)) {
          // This section updates all relevant variables from the received RF packet
          switch(game.command) {
              case NOP   : // No special operation, run game as normal
                  // Do not permit normal operations if manual override, pause or stop are in effect
                  if (curr_command == MANUAL_OVERRIDE || curr_command == STOP || curr_command == PAUSE) { 
                    if (DEBUG_RF){Serial.println("RF: NOP UNAVAILABLE");}
                    break;
                  }
                  if (DEBUG_RF){Serial.println("RF: NOP ISSUED");}
                  curr_command = NOP;
                  break;
              case START : // Begin the game from the initial position after the end of a game
                  // Cannot start game unless robots are in the STOP state. (STOP can be issued at any time)
                  if (curr_command != STOP) {
                      if (DEBUG_RF){Serial.println("RF: START UNAVAILABLE");}
                      break; 
                  }
                  // Turn on all robots, ghosts must cascade out of middle section
                  // Return to normal play
                  // NOTE: Not sure if you want to do special motor control stuff using the START command
                  //       if so, just change this to START and add a case to the very next switch statement
                
                  curr_command = START;
                  if (DEBUG_RF){Serial.println("RF: START ISSUED");}
                  break;
            case STOP  : // To be sent when the game is over
                if ((game.override_dir & PAC_DEATH) == PAC_DEATH) {
                    if (DEBUG_RF){Serial.println("RF: PACMAN DIED");}
                    // Call pacman death sequence
                }
                // Turn off all robots
                // Remain stopped
                if (DEBUG_RF){Serial.println("RF: STOP ISSUED");}
                curr_command = STOP;
                break; 
            case PAUSE : // Pause the game temporarily
                // Prevents sneaky resume of gameplay (STOP -> PAUSE -> RESUME, would be valid otherwise)
                if (curr_command == STOP) {
                    if (DEBUG_RF){Serial.println("RF: PAUSE UNAVAILABLE");}
                  break;
                }
                if (DEBUG_RF){Serial.println("RF: PAUSE ISSUED");}
                // Turn off all robot's motors, probably not the lights
                // Remain paused until resume issued
                curr_command = PAUSE;
                break;
            case RESUME:  // Resume from a pause
                // Turn on all robot's motors
                // CANNOT resume from a stop
                if (curr_command == STOP) {
                    if (DEBUG_RF){Serial.println("RF: RESUME UNAVAILABLE");}
                  break;
                }
                if (DEBUG_RF){Serial.println("RF: RESUME ISSUED");}
                // Return to normal play
                curr_command = NOP;
                break;
            case MANUAL_OVERRIDE  :
                // Can't control while paused or stopped.
                if (curr_command == STOP || curr_command == PAUSE) {
                    if (DEBUG_RF){Serial.println("RF: MANUAL_OVERRIDE UNAVAILABLE");}
                  break;
                }
                // Can't manually control ghosts
                if (currPlayer != PACMAN) {
                    if (DEBUG_RF){Serial.println("RF: CANNOT MANUAL_OVERRIDE GHOSTS");}
                    break; 
                }
                
                if (DEBUG_RF){Serial.println("RF: MANUAL_OVERRIDE ISSUED");}
                // Set current time, only use manual override
                manual_override_timer = millis();
                curr_command = MANUAL_OVERRIDE;
                player_direction = game.override_dir;
                break;
            default    : 
                if (DEBUG_RF){Serial.println("RF: INVALID COMMAND");}
                break; //Do nothing 
          }
        }
        // Clear the game header after use
        game.header = -1;
        // The meat of controlling the motors + line sensing will be done here
        uint8_t options = 0;
        switch(curr_command) {
          case NOP             : /*Execute normal AI control functions*/ 
            // TODO Currently does not do AI
            // Continue to the next case... 
          case MANUAL_OVERRIDE : /*Use game.override_dir (U, D, L, R) to decide your next move*/ 
            if (curr_command == MANUAL_OVERRIDE) {
               rgb[0] = 0;
               rgb[1] = 220;
               rgb[2] = 0;
            }
            Serial.println("DetectIntersection...");
            options = DetectIntersection();
            Serial.println("DetectIntersection - done");
            if (options > 0) {
              DecideDirection(options);
            } else {
              //globalHeading = '0'; 
            }
            if(globalHeading!='0'){
              LineFollow();
            }
            /*if(moveFlag){
              moveFlag = 0;
              MoveRobot();
            }*/
            break;
          case PAUSE           : /*Continue to the next case...*/
          case STOP            : /*If button pressed, change player*/ break;
          default              : /*PAUSE, STOP, START*/ break;
        }
    
    #else  
        //    // Non-map based motor control goes here
        //    //red = 0; green = 0; blue = 0;
        //    //DebugColor();
        //   // DecideDirection(DetectIntersection());
        //    uint8_t options;
        //    
        //    if ((options = DetectIntersection()) > 0) {
        //      DecideDirection(options);
        //    }
        //    LineFollow();
        //    /*if(moveFlag){
        //      moveFlag = 0;
        //      MoveRobot();
        //    }*/
    #endif
  
    if(millis()-loopTime>500){
        if (currPlayer != PACMAN) {
            GhostAnimation();
        } else {
            PacmanAnimation();
        }
        loopTime = millis();
    }
}

//TODO Animates Colors on Robot as Pacman?
void PacmanAnimation(){
  uint8_t pacLEDsegment;
  int i;
  for(i=0;i<20;i++){
    pixels.setPixelColor(i, pixels.Color(rgb[0],rgb[1],rgb[2]));
  }
  if(animationToggle==0){
    animationToggle = 1;
  } else {
    animationToggle = 0;
    switch(globalHeading){
      case 'u':
        pacLEDsegment = 0;
        break;
      case 'r':
        pacLEDsegment = 1;
        break;
      case 'd':
        pacLEDsegment = 2;
        break;
      case 'l':
        pacLEDsegment = 3;
        break;
      default:
        pixels.show();
        return;
        break;
    }
    for(i=0;i<5;i++){
      pixels.setPixelColor(pacStrip[pacLEDsegment][i],0,0,0);
    }
  }
  pixels.show();
}
              
//TODO Ghost Color Animations? Also Manages Eyes direction
void GhostAnimation(){
  heading_t ledHeading = globalHeading;
  if(ledHeading == 'u' || ledHeading == 'd') {
      if(animationToggle==0){
          animationToggle = 1;
          ledHeading='r';
      } else {
          animationToggle = 0;
          ledHeading='l';
      }
  }
  if(ledHeading=='r'){
    digitalWrite(EYE_LR,HIGH);
    digitalWrite(EYE_RR,HIGH);
    digitalWrite(EYE_LL,LOW);
    digitalWrite(EYE_RL,LOW);
  } else if(ledHeading=='l'){
    digitalWrite(EYE_LL,HIGH);
    digitalWrite(EYE_RL,HIGH);
    digitalWrite(EYE_LR,LOW);
    digitalWrite(EYE_RR,LOW);
  } else {
    digitalWrite(EYE_LR,HIGH);
    digitalWrite(EYE_RL,HIGH);
    digitalWrite(EYE_LL,LOW);
    digitalWrite(EYE_RR,LOW);
  }
}



/*
 *
 * RADIO NETWORK FUNCTIONS 
 *
 */
 //TODO Purpose
void UpdateGame() {
    int i;
    game_state_t game_buf;
    //noInterrupts();
    Serial.println("UpdateGame...");
    readingRadio = 1;
    if (DEBUG_RF){Serial.println("Updating Game...");}
    if (radio.available()) {
        radio.read((char *)&game_buf,GAME_SIZE);
        if (game_buf.header == GetChecksum(&game_buf) && game_buf.command == MANUAL_OVERRIDE) {
             game.command = MANUAL_OVERRIDE;
             game.override_dir = game_buf.override_dir;
             // Do NOT overwrite current pac and ghost locations and headings
             // A REALLY dodgy way to keep the game consistent, because the controller
             // doesn't have robot information
             SetChecksum();
        } else if (game_buf.header == GetChecksum(&game)) {
            // For normal game update, 
            game = game_buf;
        }
    } else {
        // Not achievable with GetChecksum()
        game.header = -1;
    }
    readingRadio = 0;
    //  lastRadioUpdateTime = millis();
    //interrupts();
    radio_flag = 0;
    Serial.println("UpdateGame - done");
}

//Get Checksum of a game_state_t
uint8_t GetChecksum(game_state_t *g) {
    Serial.println("GetChecksum...");
    uint8_t i, j, sum = 0;
    for (i = 1; i < GAME_SIZE; i++) {
        for (j = 0; j < 8; j++) {
            sum += (((char *)g)[i] & (1 << j)) >> j;
        }
    }
    Serial.println("GetChecksum - done");
    return sum;
}

//Set checksum of game variable
void SetChecksum() {
    Serial.println("SetChecksum...");
    uint8_t i, j, sum = 0;
    for (i = 1; i < GAME_SIZE; i++) {
        for (j = 0; j < 8; j++) {
            sum += (((char *)&game)[i] & (1 << j)) >> j;
        }
    }
    game.header = sum;
    Serial.println("SetChecksum - done");
}



/*
 *
 * MOVEMENT FUNCTIONS 
 *
 */
//Used in Setup and Loop. TODO Purpose
void InitHeading(){
    Serial.println("InitHeading...");
    line_pos_t readTop = lineTop.get_line();
    line_pos_t readRight = lineRight.get_line();
    line_pos_t readBottom = lineBottom.get_line();
    line_pos_t readLeft = lineLeft.get_line();
    if(readTop!=NONE){
        globalHeading = 'u';
    } else if(readRight!=NONE){
        globalHeading = 'r';
    } else if(readLeft!=NONE){
        globalHeading = 'l';
    } else if(readBottom!=NONE){
        globalHeading = 'd';
    } else {
        //something is probably wrong
        globalHeading = 'u';
    }
    Serial.println("InitHeading - done");
}

//TODO Purpose
void MoveRobot() {
    Serial.println("MoveRobot...");
    if(readingRadio){
        return;
    }
    if(currentHeading!=globalHeading){
        if(currentHeading=='0'){
            m_topLeft.enableOutputs();
            m_topRight.enableOutputs();
            m_bottomLeft.enableOutputs();
            m_bottomRight.enableOutputs();
        }
        currentHeading=globalHeading;  
    }
    if(globalHeading=='0'){
        UpdateSpeed(&m_topLeft,0);
        UpdateSpeed(&m_topRight,0);
        UpdateSpeed(&m_bottomLeft,0);
        UpdateSpeed(&m_bottomRight,0);
        m_topLeft.disableOutputs();
        m_topRight.disableOutputs();
        m_bottomLeft.disableOutputs();
        m_bottomRight.disableOutputs();
    }
    m_topLeft.runSpeed();
    m_topRight.runSpeed();
    m_bottomLeft.runSpeed();
    m_bottomRight.runSpeed();
    Serial.println("MoveRobot - done");
}

//TODO Purpose
void UpdateSpeed(AccelStepper *thisMotor,int newSpeed){
    Serial.println("update_speed...");
    int speedVal = (int)thisMotor->speed();
    if(speedVal!=newSpeed){
      thisMotor->setSpeed(newSpeed);

    }
    Serial.println("update_speed - done");
}

// TODO Purpose
void GetMotorAlignment(AccelStepper **m_fL,AccelStepper **m_fR,AccelStepper **m_bL,AccelStepper **m_bR){
    // global heading must have motor direction first
    Serial.println("GetMotorAlignment...");
    switch(globalHeading){
        case 'u' :
            *m_fL = &m_topLeft;
            *m_fR = &m_topRight;
            *m_bL = &m_bottomLeft;
            *m_bR = &m_bottomRight;
            break;
        case 'r' :
            *m_fL = &m_topRight;
            *m_fR = &m_bottomRight;
            *m_bL = &m_topLeft;
            *m_bR = &m_bottomLeft;
            break;
        case 'd' :
            *m_fL = &m_bottomRight;
            *m_fR = &m_bottomLeft;
            *m_bL = &m_topRight;
            *m_bR = &m_topLeft;
            break;
        case 'l' :
            *m_fL = &m_bottomLeft;
            *m_fR = &m_topLeft;
            *m_bL = &m_bottomRight;
            *m_bR = &m_topRight;
            break;
        default : 
            *m_fL = &m_topLeft;
            *m_fR = &m_topRight;
            *m_bL = &m_bottomLeft;
            *m_bR = &m_bottomRight;
            break;
    }
    Serial.println("GetMotorAlignment - done");
}

// TODO: Add explanation
void GetLineAlignment(LineSensor **l_F,LineSensor **l_B){
    Serial.println("GetLineAlignment...");
    switch(globalHeading){
    case 'u' :
        *l_F = &lineTop;
        *l_B = &lineBottom;
        break;
    case 'r' :
        *l_F = &lineRight;
        *l_B = &lineLeft;
        break;
    case 'd' :
        *l_F = &lineBottom;
        *l_B = &lineTop;
        break;
    case 'l' :
        *l_F = &lineLeft;
        *l_B = &lineRight;
        break;
    default : 
        *l_F = &lineTop;
        *l_B = &lineBottom;
        break;
    }
    Serial.println("GetLineAlignment - done");
}

// TODO: Add explanation
void LineFollow(){
    Serial.println("LineFollow...");
    // use of pointers helps translate robot movement functions based on direction
    AccelStepper *m_frontLeft;
    AccelStepper *m_frontRight;
    AccelStepper *m_backLeft;
    AccelStepper *m_backRight;
    LineSensor *lineFront;
    LineSensor *lineBack;

    GetMotorAlignment(&m_frontLeft,&m_frontRight,&m_backLeft,&m_backRight);
    GetLineAlignment(&lineFront,&lineBack);
    line_pos_t readFront = lineFront->get_line();
    line_pos_t readBack = lineBack->get_line();

    if(readFront!=NONE&&readBack!=NONE){ // dual sensor control
        //front pair of motors
        if(readFront==RIGHT){
            UpdateSpeed(m_frontLeft,M_SPEED);
            UpdateSpeed(m_frontRight,-M_SLOW);
        } else if(readFront==LEFT){
            UpdateSpeed(m_frontLeft,M_SLOW);
            UpdateSpeed(m_frontRight,-M_SPEED);
        } else {
            UpdateSpeed(m_frontLeft,M_SPEED);
            UpdateSpeed(m_frontRight,-M_SPEED);
        }
        //back pair of motors
        if(readBack==RIGHT){
            UpdateSpeed(m_backLeft,M_SPEED);
            UpdateSpeed(m_backRight,-M_SLOW);
        } else if(readBack==LEFT){
            UpdateSpeed(m_backLeft,M_SLOW);
            UpdateSpeed(m_backRight,-M_SPEED);
        } else {
            UpdateSpeed(m_backLeft,M_SPEED);
            UpdateSpeed(m_backRight,-M_SPEED);
        }
    } else { // one sensor must shift whole robot
        if(readFront==RIGHT||readBack==LEFT){
            UpdateSpeed(m_frontLeft,M_SPEED);
            UpdateSpeed(m_backRight,-M_SPEED);
            UpdateSpeed(m_frontRight,-M_SLOW);
            UpdateSpeed(m_backLeft,M_SLOW);
        } else if(readFront==LEFT||readBack==RIGHT){
            UpdateSpeed(m_frontRight,-M_SPEED);
            UpdateSpeed(m_backLeft,M_SPEED);
            UpdateSpeed(m_frontLeft,M_SLOW);
            UpdateSpeed(m_backRight,-M_SLOW);
        } else {
          //lost the line
            UpdateSpeed(m_frontLeft,M_SPEED);
            UpdateSpeed(m_backLeft,M_SPEED);
            UpdateSpeed(m_frontRight,-M_SPEED);
            UpdateSpeed(m_backRight,-M_SPEED);
        }
    }
    Serial.println("LineFollow - done");
}

// TODO: Add explanation
bool AlignToIntersection() {
    Serial.println("AlignToIntersection...");
    if(globalHeading=='0'){
       return true;
    }
    int m_tL_speed = 0;
    int m_tR_speed = 0;
    int m_bL_speed = 0;
    int m_bR_speed = 0;
    bool ret = true;
    line_pos_t readTop = lineTop.get_line();
    line_pos_t readRight = lineRight.get_line();
    line_pos_t readBottom = lineBottom.get_line();
    line_pos_t readLeft = lineLeft.get_line();
    if(readTop==LEFT||readTop==RIGHT){
        ret = false;
        if(readTop==LEFT){
          m_tL_speed-=M_SPEED;
          m_tR_speed-=M_SPEED;
        } else {
          m_tL_speed+=M_SPEED;
          m_tR_speed+=M_SPEED;
        }
    }
    if(readRight==LEFT||readRight==RIGHT){
        ret = false;
        if(readRight==LEFT){
          m_tR_speed-=M_SPEED;
          m_bR_speed-=M_SPEED;
        } else {
          m_tR_speed+=M_SPEED;
          m_bR_speed+=M_SPEED;
        }
    }
    if(readBottom==LEFT||readBottom==RIGHT){
        ret = false;
        if(readBottom==LEFT){
          m_bR_speed-=M_SPEED;
          m_bL_speed-=M_SPEED;
        } else {
          m_bR_speed+=M_SPEED;
          m_bL_speed+=M_SPEED;
        }
    }
    if(readLeft==LEFT||readLeft==RIGHT){
        ret = false;
        if(readLeft==LEFT){
          m_bL_speed-=M_SPEED;
          m_tL_speed-=M_SPEED;
        } else {
          m_bL_speed+=M_SPEED;
          m_tL_speed+=M_SPEED;
        }
    }
    UpdateSpeed(&m_topLeft,m_tL_speed);
    UpdateSpeed(&m_topRight,m_tR_speed);
    UpdateSpeed(&m_bottomLeft,m_bL_speed);
    UpdateSpeed(&m_bottomRight,m_bR_speed);
    Serial.println("AlignToIntersection - done");
    return ret;
}  

//TODO To be Completed
uint8_t DetectIntersection() {
  // returning true when robot has just reached an intersection
  // (TODO to be completed)
  if((millis()-lastIntersection)<2000&&globalHeading!='0'){
    return 0;
  }
  line_pos_t readTop = lineTop.get_line();
  line_pos_t readRight = lineRight.get_line();
  line_pos_t readBottom = lineBottom.get_line();
  line_pos_t readLeft = lineLeft.get_line();
  uint8_t trip = (readTop!=NONE)+(readRight!=NONE)+(readBottom!=NONE)+(readLeft!=NONE);
  if (trip<2) {
    return 0;
  } else if(trip>2) {
    // red
    //red = 50; green = 0; blue = 0;
    //DebugColor();
    unsigned long time = millis();
    while(!AlignToIntersection() && (millis() - time < MAX_ALIGN_TIME)){}
  } else if(readTop != NONE && readBottom != NONE) {
    return 0;
  } else if(readLeft != NONE && readRight != NONE) {
    return 0;
  } else {
    unsigned long time = millis();
    while(!AlignToIntersection() && (millis() - time < MAX_ALIGN_TIME)){}
  }  
  uint8_t options = 0;
  readTop = lineTop.get_line();
  readRight = lineRight.get_line();
  readBottom = lineBottom.get_line();
  readLeft = lineLeft.get_line();
  trip = (readTop!=NONE)+(readRight!=NONE)+(readBottom!=NONE)+(readLeft!=NONE);
  if (trip<2) {
    return 0;
  } else if(trip>2) {
    //red = 50; green = 0; blue = 0;
    //DebugColor();
    if(readTop!=NONE){
      options|=U;
    }
    if(readRight!=NONE){
      options|=R;
    }
    if(readBottom!=NONE){
      options|=D;
    }
    if(readLeft!=NONE){
      options|=L;
    }
    lastIntersection = millis();
    return options;
  } else if(readTop!=NONE&&readBottom!=NONE){
    return 0;
  } else if(readLeft!=NONE&&readRight!=NONE){
    return 0;
  } else {
    if(currPlayer==PACMAN){
      // treat as intersection
      if(readTop!=NONE){
        options|=U;
      }
      if(readRight!=NONE){
        options|=R;
      }
      if(readBottom!=NONE){
        options|=D;
      }
      if(readLeft!=NONE){
        options|=L;
      }
      lastIntersection = millis();
      return options;
    } else {
      // do not treat as intersection
      if(readTop!=NONE&&globalHeading!='d'){
        globalHeading='u';
      } else if(readRight!=NONE&&globalHeading!='l'){
        globalHeading='r';
      } else if(readBottom!=NONE&&globalHeading!='u'){
        globalHeading='d';
      } else if(readLeft!=NONE&&globalHeading!='r'){
        globalHeading='l';
      }
      lastIntersection = millis();
      return 0;
    }
  }
}

//TODO Purpose
void DecideDirection(uint8_t options){  
    Serial.println("DecideDirection...");
    #ifdef PRINT_DECISION
        Serial.print("Global heading: ");
        Serial.println((char)globalHeading);
        Serial.print("Passed in line options: ");
        Serial.println(options,BIN);
        Serial.print("Manual player request heading: ");
        Serial.println(player_direction,BIN);
    #endif
    heading_t newHeading;
    heading_t directionList[4];
    uint8_t directionInts[4];
    float angle;
    // Make doubling back impossible. (maybe enable for pacman later)
    heading_t opposite_heading;
    switch(globalHeading) {
      case 'u': opposite_heading = 'd'; break;
      case 'd': opposite_heading = 'u'; break;
      case 'l': opposite_heading = 'r'; break;
      case 'r': opposite_heading = 'l'; break;
      default : opposite_heading = '0'; break; 
    }
    // Allow player to turn back on themselves
    if (curr_command != MANUAL_OVERRIDE) {
      options &= ~HeadingToBinary(opposite_heading);
    }
    //Serial.print("After removing opposite heading: ");
    //Serial.println(options,BIN);
    goal.x = 0;
    goal.y = 0;
    #ifdef USE_RADIO
    if (curr_command == MANUAL_OVERRIDE) {
      if ((player_direction & options) > 0) {
        Serial.print("player's direction available :");
        // If the override direction is available, set that heading
        globalHeading = BinaryToHeading(player_direction);
        Serial.println((char)globalHeading);
        player_direction  = 0;
      } else if ((HeadingToBinary(globalHeading) & options)>0) {
        Serial.println("player's direction not available, but global is");
        // If the current direction is available, maintain
        globalHeading = globalHeading; // lol
      } else {
        Serial.println("player nor global available");
        globalHeading = '0';
        player_direction = 0;
      }
        Serial.println("DecideDirection - done");

      return;
    }
    // This else is to
    #endif
    if (goal.x == 0 && goal.y == 0) { //random movement mode
        char randPriority[5] = "udlr";
        uint8_t tempDir;
        uint8_t randSelect;
        uint8_t valid  = 0;
        while(valid==0){
          Serial.println("invalid decision");
          randSelect = millis()%4;
          //Serial.print("rand select = ");
          //Serial.println(randSelect);
          
          newHeading = randPriority[randSelect];
          //Serial.println((char)newHeading);
          tempDir = HeadingToBinary(newHeading);
          valid = ((tempDir&options)>0);
          //delay(200);
        }
        globalHeading = newHeading;
        Serial.println("DecideDirection - done");

        return;  
        //    newHeading = opposite_heading;
        //    while (newHeading == opposite_heading) {
        //       headingList[4]
        //    }
        //      // heading is going to be a single bit set
        //     do {
        //      uint8_t headingList[4] = {U,L,R,D};
        //      uint8_t heading = options & headingList(random(0,4));
        //      if (heading != 0) {
        //        newHeading = BinaryToHeading(heading);
        //      }
        //    } while (newHeading == 0);
        //    globalHeading = newHeading;
        //    return;
    } else if(thisRobot->p.y==goal.y&&thisRobot->p.x==goal.x){
      switch(thisRobot->h){
        case 'u':
          angle = 90;
          break;
        case 'r':
          angle = 0;
          break;
        case 'd':
          angle = -90;
          break;
        case 'l':
          angle = 180;
          break;
        default :
          angle = 90;
          break;
      }
    } else {
      angle = atan2(thisRobot->p.y-goal.y,goal.x-thisRobot->p.x);
    } 
    if (angle>=135) {
      strncpy(directionList,"ludr",4);
    } else if(angle>=90){
      strncpy(directionList,"ulrd",4);
    } else if(angle>=45){
      strncpy(directionList,"urld",4);
    } else if(angle>=0){
      strncpy(directionList,"rudl",4);
    } else if(angle>=-45){
      strncpy(directionList,"rdul",4);
    } else if(angle>=-90){
      strncpy(directionList,"drlu",4);
    } else if(angle>=-135){
      strncpy(directionList,"dlru",4);
    } else if(angle>=-180){
      strncpy(directionList,"ldur",4);;
    } else { // default probably not needed
      strncpy(directionList,"urdl",4);
    }
    int i;
    for(i=0;i<4;i++){
      directionInts[i] = HeadingToBinary(directionList[i]);
    }
    i=0;
    while(i<4){
      if((options&directionInts[i])>0){
        break;
      }
      i++;
    }
    if(i==4){
      //newHeading = '0';
      globalHeading = '0';
    } else {
      //newHeading = directionList[i];
      globalHeading = directionList[i];
    }  
    Serial.println("DecideDirection - done");
    return
}

//TODO is this function even used
void MoveFlag(){
  Serial.println("MoveFlag...");
  moveFlag = 1;
  Serial.println("MoveFlag - done");
}



/*
 *
 * MAP NAVIGATION FUNCTIONS 
 *
 */
//TODO Purpose
void SetGoal(){
    switch(currPlayer){
        case PACMAN:
            break;
        case GHOST1:
            if(aiTime-millis()<15000){
                goal.x = 1;
                goal.y = 1;
            } else if(aiTime-millis()<35000){
                goal.x = game.pac.p.x;
                goal.y = game.pac.p.y;
            } else {
                aiTime = millis();
            }
            break;
        case GHOST2:
            if(aiTime-millis()<15000){
                goal.x = 9;
                goal.y = 9;
            } else if(aiTime-millis()<35000){
                goal.x = game.pac.p.x;
                goal.y = game.pac.p.y;
                int newX = goal.x;
                int newY = goal.y;
                CalcNewSquare(&newX,&newY,game.pac.h,5);
                MapConstrain(&newX,&newY);
                goal.x = newX;
                goal.y = newY;
            } else {
                aiTime = millis();
            }
            break;
        case GHOST3:
            if(aiTime-millis()<15000){
                goal.x = 9;
                goal.y = 1;
            } else if(aiTime-millis()<35000){
                goal.x = random(1,9);
                goal.y = random(1,9);
            } else {
                aiTime = millis();
            }
            break;
        default:
            break;
    }
}

//TODO Where is this function used?
uint8_t ExpandMap(){
    int x = thisRobot->p.x;
    int y = thisRobot->p.y;
    char h = thisRobot->h;
    uint8_t *r;
    uint8_t options = 0b0000;
    int tempX, tempY;
    // expands along 
    if(IsOpen(x,y,'u')){
        tempX = x; tempY = y-1;
        r = Expand(&tempX,&tempY,'u');
        if(r==NULL) {
            if(!CollisionDetect('u')){
                //Serial.println("Up path available");
                options|=U;
            }
        }
    }
    if(IsOpen(x,y,'r')){
        tempX = x+1; tempY = y;
        r = Expand(&tempX,&tempY,'r');
        if(r==NULL) {
            if(!CollisionDetect('r')){
                //Serial.println("Right path available");
                options|=R;
            }
        }
    }
    if(IsOpen(x,y,'d')){
        tempX = x; tempY = y+1;
        r = Expand(&tempX,&tempY,'d');
        if(r==NULL) {
            if(!CollisionDetect('d')){
                //Serial.println("Down path available");
                options|=D;
            }
        }
    }
    if(IsOpen(x,y,'l')){
        tempX = x-1; tempY = y;
        r = Expand(&tempX,&tempY,'l');
        if(r==NULL) {
            if(!CollisionDetect('l')){
                //Serial.println("Left path available");
                options|=L;
            }
        }
    }
    return(options);
}

/*
 * Checks whether the position on the map contains 3 or more directions to move in
 * Returns 1 if true, 0 if false.
*/
bool IsIntersection(int x, int y) {
    if(x < 1 || x > 9 || y < 1 || y > 9) {
        return false;
    }
    uint8_t square = game_map[y-1][x-1];
    int dirCount = 0;
    dirCount += (square & U) > 0;
    dirCount += (square & R) > 0;
    dirCount += (square & D) > 0;
    dirCount += (square & L) > 0;
    return (dirCount>2);
}

//TODO Purpose
bool IsOpen(int x, int y, heading_t dir){
    if(x<1||x>9||y<1||y>9){
        return false;
    }
    uint8_t square = game_map[y-1][x-1];
    uint8_t testDir = HeadingToBinary(dir);
    return ((square&testDir)>0);
}

//TODO Purpose
bool CollisionDetect(heading_t h){
    robot_t *r;
    bool collision = false;
    int xDest = thisRobot->p.x;
    int yDest = thisRobot->p.y;
    if (IsIntersection(xDest,yDest)){ 
        // force look at the next intersection
        if(IsOpen(xDest,yDest,h)){
            CalcNewSquare(&xDest,&yDest,h,1);
        }
    }
    ExpandSingle(&xDest,&yDest,h);
    // possibility to expand a second time if need be
    if (!IsIntersection(xDest,yDest)){
        return false; // if we're not about to reach intersection, don't worry!
        // may want to alter behaviour to keep spacing in future
    }
    int i;
    int xDestCheck, yDestCheck;
    heading_t hCheck;
    for(i=0;i<NUM_GHOSTS;i++){
        if(i!=currPlayer-1){ //don't check with yourself
            r = &game.g[i];
            xDestCheck = r->p.x;
            yDestCheck = r->p.y;
            hCheck = r->h;
            if(IsIntersection(xDestCheck,yDestCheck)){
                if(xDestCheck==xDest&&yDestCheck==yDest){
                    //stop our ghost robot from colliding!
                    //need to insert relevant action (i know what i want to do, just have to do it)
                    Serial.println("Stop the bot!");
                    collision = true;
                    break;
                }
            } else {
                ExpandSingle(&xDestCheck,&yDestCheck,hCheck);
                if(xDestCheck==xDest&&yDestCheck==yDest){
                    if(i<currPlayer-1){
                        //the incoming ghost is a higher priority! let them go first!
                        Serial.println("Stop the bot!");
                        collision = true;
                        break;
                    } else {
                        // we want to go through the intersection first, assume they will stop
                    }
                }
            }
        }
    }
    return collision;
}
 
//TODO Purpose 
uint8_t* CheckSquare(int x,int y){
    robot_t *r = NULL;
    if(x<1||x>9||y<1||y>9){
        r = NULL;
    } else if (game.pac.p.x==x&&game.pac.p.y==y){
        r = &game.pac;
    } else {  // probably shouldn't consider pacman's location
        int i;
        for(i = 0; i < NUM_GHOSTS; i++){
            if (game.g[i].p.x == x && game.g[i].p.y ==y){
                r = &game.g[i];
                break;
            }
        }
    }
    return (uint8_t*)r;
}

//TODO Purpose
uint8_t* Expand(int *x,int *y, heading_t h){
    // Debug output
    //Serial.print("Checking: ");
    //Serial.print(*x); Serial.print(" "); Serial.println(*y);
    if(*x<1||*x>9||*y<1||*y>9){
        return NULL;
    }
    uint8_t * r = CheckSquare(*x,*y);
    if(r!=NULL){
        // return r;
        if(r==(uint8_t*)&game.pac){
            r=NULL;
        }
    } else if(!IsIntersection(*x,*y)) {
        // Start potentially redundant code
        // Checks to expand along direction robot is heading first
        if(IsOpen(*x,*y,h)){
            CalcNewSquare(x,y,h,1);
            r = Expand(x,y,h);
        } // end potentially redundant code 
        else if(IsOpen(*x,*y,'u')&&h!='d'){
            *y = *y-1;
            r = Expand(x,y,'u');
        } else if(IsOpen(*x,*y,'r')&&h!='l'){
            *x = *x+1;
            r = Expand(x,y,'r');
        } else if(IsOpen(*x,*y,'d')&&h!='u'){
            *y = *y+1;
            r = Expand(x,y,'d');
        } else if(IsOpen(*x,*y,'l')&&h!='r'){
            *x = *x+1;
            r = Expand(x,y,'l');
        }
    }
    return r;
}

//TODO Purpose
uint8_t* ExpandSingle(int *x,int *y,heading_t h){
    if(*x<1||*x>9||*y<1||*y>9){
        return NULL;
    }
    uint8_t * r = NULL;//CheckSquare(*x,*y);
    if(!IsIntersection(*x,*y)){
        // Start potentially redundant code
        // Checks to expand along direction robot is heading first
        if(IsOpen(*x,*y,h)){
            CalcNewSquare(x,y,h,1);
            r = CheckSquare(*x,*y);
        } // end potentially redundant code 
        else if(IsOpen(*x,*y,'u')&&h!='d'){
            *y = *y-1;
            r = CheckSquare(*x,*y);
        } else if(IsOpen(*x,*y,'r')&&h!='l'){
            *x = *x+1;
            r = CheckSquare(*x,*y);
        } else if(IsOpen(*x,*y,'d')&&h!='u'){
            *y = *y+1;
            r = CheckSquare(*x,*y);
        } else if(IsOpen(*x,*y,'l')&&h!='r'){
            *x = *x+1;
            r = CheckSquare(*x,*y);
        }
    }
    if(r==(uint8_t*)&game.pac){
        r=NULL;
    }
    return r;
}


//TODO Purpose
void CalcNewSquare(int *x,int*y,heading_t h,int d){
  uint8_t dir = HeadingToBinary(h);
  *x = *x + ((xx&dir&addDir)>0)*d - ((xx&dir&subDir)>0)*d;
  *y = *y + ((yy&dir&addDir)>0)*d - ((yy&dir&subDir)>0)*d;
}

uint8_t HeadingToBinary(heading_t h){
    uint8_t ret;
    switch(h){
        case 'u' : ret = U; break;
        case 'r' : ret = R; break;
        case 'd' : ret = D; break;
        case 'l' : ret = L; break;
        default  : ret = 0b0000; break;
    }
    return(ret);
}

char BinaryToHeading(uint8_t h) {
    char ret;
    switch(h){
        case U : ret = 'u'; break;
        case R : ret = 'r'; break;
        case D : ret = 'd'; break;
        case L : ret = 'l'; break;
        default  : ret = '0'; break;
    }
    return(ret);
  
}

//TODO What is constrain. Used only once
void MapConstrain(int *x,int *y){
    *x = constrain(*x,1,9);
    *y = constrain(*y,1,9);
}



//Debug
void DebugColor() {
    int i;
    for(i=0;i<NUMPIXELS;i++) {
      pixels.setPixelColor(i, pixels.Color(red,green,blue));
    }
    pixels.show();
}

//Debug
void PrintGame() {
     int i;
     for (i = 0; i < GAME_SIZE; i++) {
        Serial.print("Byte \0");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(*((char *)&game+i), DEC);
     }
     Serial.println();
     Serial.println();Serial.println();
}

// test function only
void enter_robot_location(robot_t * entry){
    int x;
    int y;
    char temp;
    heading_t h;
    Serial.println("Please enter x y heading in format xyh");
    while(!Serial.available()){}
    temp = Serial.read();
    x = atoi(&temp);
    Serial.print(x);
    while(!Serial.available()){}
    temp = Serial.read();
    y = atoi(&temp);
    Serial.print(y);
    while(!Serial.available()){}
    h = Serial.read();
    Serial.println(h);
    //Test locations
    entry->p.x = x;
    entry->p.y = y;
    entry->h = h;
}
