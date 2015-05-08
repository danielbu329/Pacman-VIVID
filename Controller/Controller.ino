#include <SPI.h>
#include "RF24.h"

/*
 * PACMAN RF Data Struct and utilities
 */

#include "RF24.h"

/*
 * Game Struct definitions
 */

typedef struct _position {
    uint8_t x : 4;
    uint8_t y : 4;
} position;

typedef char heading;

typedef struct _robot {
    position p;
    heading h;
} robot;

// Contains information about all robots currently in play.
typedef struct _game_state {
    uint8_t header;
    uint8_t command;
    uint8_t override_dir;	
    robot pac;
    robot g1;
    robot g2;
    robot g3;
} game_state;

/*
 * Game state definitions
 */ 
// Header
#define HEADER 0xC8

// Game commands
#define NOP      0
#define START    1
#define STOP     2
#define HIDE     3
#define PAUSE    4
#define RESUME   5 
#define RUN_MODE 6
#define MANUAL_OVERRIDE 7

#define GAME_SIZE sizeof(game_state)

/*
 * Controller definitions
 */
 
#define U 0b1000
#define R 0b0100
#define D 0b0010
#define L 0b0001

/*
 * Radio definitions
 */

#define BROADCAST_CHANNEL 72

/*
 * Functions to be used by robots and hub
 */

void init_radio(void);
void init_game(void);
void broadcast_game(void);

/*
 * Global Variables
 */
 // Only need a single pipe due to one-way comms.
const uint64_t pipe = 0xF0F0F0F0E1LL;
RF24 radio(9,10);
int time;
game_state game;
// Pointer to be used when updating game from radio
game_state *g = &game;
// Pin definitions
int up    = 3;
int down  = 4;
int left  = 5;
int right = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_radio();
  init_game();
  pinMode(up   , INPUT);
  pinMode(down , INPUT);
  pinMode(left , INPUT);
  pinMode(right, INPUT);
  Serial.println("Now receiving inputs!");
}

void loop() {
  game.command = MANUAL_OVERRIDE;
  // Read button presses
  if (digitalRead(up)) 
    game.override_dir = U;
  else if (digitalRead(down))
    game.override_dir = D;
  else if (digitalRead(left))
    game.override_dir = L;
  else if (digitalRead(right))
    game.override_dir = R;
  else {
    game.override_dir = 0;
    game.command = NOP;
  }
  // Send game information
  broadcast_game();
}

void init_game() {
  // All data should be out of bounds for the controller  
  game.command = NOP;
  game.override_dir = 0;
  game.pac = {-1,-1};
  game.g1 =  {-1,-1};
  game.g2 =  {-1,-1};
  game.g3 =  {-1,-1};
}

void init_radio() {
  radio.begin();
  radio.setRetries(0,0);
  radio.openWritingPipe(pipe);
  radio.setChannel(BROADCAST_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.stopListening();
}

// Used by Matlab connected arduino 
void broadcast_game() {
  int i;
  /*
  for (i = 0; i < GAME_SIZE; i++) {
    Serial.println(buf[i], DEC);
    Serial.print("Byte ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(((char *)g)[i], DEC);
  }
  Serial.println();
  */
  // This increment loop avoids modifying the checksum
  for (i = 1; i < GAME_SIZE; i++) {
    ((char *)g)[i] += 1;
    //time = millis();
    set_checksum();
    //Serial.println(millis()- time);
    radio.write((char *)g,GAME_SIZE);  
  }
  /*if () {
     Serial.println("TX: Success");
  } else {
      Serial.println("TX: FAILURE");
  }*/
}

// A simple checksum calculator, operates within 1ms
void set_checksum(void) {
  uint8_t   i, j, sum = 0;
  // Don't include checksum
  for (i = 1; i < GAME_SIZE; i++) {
    for (j = 0; j < 8; j++) {
      sum += ((char *)g)[i] & (1 << j) >> j;
      /*Serial.print("i:");
      Serial.print(i);
      Serial.print(" j:");
      Serial.print(j);
      Serial.print(" sum:");
      Serial.println(sum);
     */ 
    }
  }
  g->header = sum;
  
}
