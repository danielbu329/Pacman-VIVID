
#ifndef Game_h
#define Game_h


/*
 * Game state definitions
 */ 
 
#define NUM_GHOSTS 3
// Header
#define HEADER 0xC8

// Game commands
#define NOP             B0000000
#define START           B0000001
#define STOP            B0000010
#define HIDE            B0000100
#define PAUSE           B0001000
#define RESUME          B0010000 
#define MUSIC_COMMAND   B0100000
#define MANUAL_OVERRIDE B1000000

#define GAME_SIZE sizeof(game_state_t)

// Pacman death sequence code, must NOT conflict with music mask of broadcaster
#define PAC_DEATH 0b10000000


/*
 * 
 *   Game Struct definitions
 *   Information on current positions of pacman and robots, header (hash), stores a command
 */

typedef struct {
    uint8_t x : 4;
    uint8_t y : 4;
} position_t;

typedef char heading_t;

typedef struct {
    position_t p;
    heading_t h;
} robot_t;

// Contains information about all robots currently in play.
typedef struct {
    uint8_t header; //Holds a checksum
    uint8_t command;
    uint8_t override_dir;   
    robot_t pac;
    robot_t g[NUM_GHOSTS];
} game_state_t;



#endif