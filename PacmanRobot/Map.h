
#ifndef Map_h
#define Map_h

/*
 * Map definitions
 */

#define DIM 9

//Binary
//0b0000 = 0, etc
#define U 0b1000
#define R 0b0100
#define D 0b0010
#define L 0b0001

// note: for error checking, we will refer to board with indices 1-9 not 0-8
// exit of "ghost cave" has been made one directional, could change easily for
// end of game or hiding mode (at x(column)=5,y(row)=4, R|L->R|D|L
static const uint8_t game_map[DIM][DIM] = {
  {R|D,   R|D|L,  R|L,    D|L,    0,     R|D,    R|L,    R|D|L,  D|L},
  {U|D,   U|R|D,  R|D|L,  U|R|L,  R|L,   U|R|L,  R|D|L,  U|D|L,  U|D},
  {U|R,   U|D|L,  U|R,    D|L,    0,     R|D,    U|L,    U|R|D,  U|L},
  {0,     U|D,    R|D,    U|R|L,  R|L,   U|R|L,  D|L,    U|D,    0  },
  {R|D,   U|R|D|L,U|D|L,  R,      U|R|L, L,      U|R|D,  U|R|D|L,D|L},
  {U|D,   U|D,    U|R,    R|D|L,  R|L,   R|D|L,  U|L,    U|D,    U|D},
  {U|R,   U|D|L,  R|D,    U|R|D|L,R|L,   U|R|D|L,D|L,    U|R|D,  U|L},
  {R|D,   U|R|L,  U|L,    U|D,    0,     U|D,    U|R,    U|R|L,  D|L},
  {U|R,   R|L,    R|L,    U|R|L,  R|L,   U|R|L,  R|L,    R|L,    U|L}
};

//Variables for smart map expansion
static const uint8_t subDir = 0b1001;
static const uint8_t addDir = 0b0110;
static const uint8_t xx = 0b0101;
static const uint8_t yy = 0b1010;

#endif