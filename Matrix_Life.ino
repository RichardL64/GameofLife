/*
  Matrix Life

  Game of life on a LED Matrix display
  Matrix Portal ESP32-S3 board
  LED Matrix output - 2x 64x64 boards (configurable)
  
  Classic Conway's Game of life rules B36/S23 (Highlife)
    1. Any live cell with fewer than two live neighbours dies, as if by underpopulation.
    2. Any live cell with two or three live neighbours lives on to the next generation.
    3. Any live cell with more than three live neighbours dies, as if by overpopulation.
    4. Any dead cell with exactly three (or 6) live neighbours becomes a live cell, as if by reproduction.

  R.Lincoln   May 2025

*/

#include <Wire.h>                 // For I2C communication
#include <Adafruit_LIS3DH.h>      // For accelerometer
#include <Adafruit_Protomatter.h> // For RGB matrix

#define HEIGHT  64  // Matrix height (pixels)
#define WIDTH   128 // Matrix width (pixels)
#define MAX_FPS 25  // Maximum redraw rate, frames/second

uint8_t rgbPins[]  = {42, 41, 40, 38, 39, 37};
uint8_t addrPins[] = {45, 36, 48, 35, 21};
uint8_t clockPin   = 2;
uint8_t latchPin   = 47;
uint8_t oePin      = 14;

#if HEIGHT == 16
#define NUM_ADDR_PINS 3
#elif HEIGHT == 32
#define NUM_ADDR_PINS 4
#elif HEIGHT == 64
#define NUM_ADDR_PINS 5
#endif

Adafruit_LIS3DH accel = Adafruit_LIS3DH();

Adafruit_Protomatter matrix( WIDTH, 5, 1, 
                             rgbPins, NUM_ADDR_PINS, addrPins,
                             clockPin, latchPin, oePin, true);


#define BLACK         matrix.color565(  0,  0,  0)
#define WHITE         matrix.color565(255,255,255)
#define DARK_GREY     matrix.color565( 16, 16, 16)
#define BROWN         matrix.color565(120, 79, 23)
#define RED           matrix.color565(228,  3,  3)
#define DARK_RED      matrix.color565( 29,  1,  1)
#define ORANGE        matrix.color565(255,140,  0)
#define DARK_ORANGE   matrix.color565( 64, 36,  0)
#define YELLOW        matrix.color565(255,237,  0)
#define BRIGHT_GREEN  matrix.color565(  0,255, 76)
#define GREEN         matrix.color565(  0,128, 38)
#define DARK_GREEN    matrix.color565(  0, 64, 19)
#define BLUE          matrix.color565(  0, 77,255)
#define DARK_BLUE     matrix.color565(  0, 20, 64)
#define PETROL_BLUE   matrix.color565(  0,  8,  8)
#define PURPLE        matrix.color565(117,  7,135)


//  Two state arrays that swap between current and next for each cycle
//
int state[2][WIDTH][HEIGHT];
int now = 0, next = 1;

//  Cellular autometa definition https://en.wikipedia.org/wiki/Life-like_cellular_automaton
//  Lookup table - current state and neighbours in gives new state out
//
struct rule {
  int colour;                         // representative colour for this state
  int alive;                          // 1 if alive for accumulating neighbour count
  int rule[9];                        // Next state/rule number indexed/selected by number of live neighbours
};

// 0  1  2  3  4  5  6  7  8          // Neighbour count
static const rule rules[] = {         // one rule per current cell state
  BLACK, 0,                           // 0 Dead
  {0, 0, 0, 1, 0, 0, 1, 0, 0},

  DARK_BLUE, 1,                       // 1 Born
  {3, 3, 2, 2, 3, 3, 3, 3, 3},

  GREEN, 1,                           // 2 Survived
  {3, 3, 2, 2, 3, 3, 3, 3, 3},

  DARK_ORANGE, 0,                     // 3 Dying
  {4, 4, 4, 1, 4, 4, 1, 4, 4},

  PETROL_BLUE, 0,                     // 4 Trail
  {4, 4, 0, 1, 0, 0, 1, 0, 0}
};
// 0  1  2  3  4  5  6  7  8


// Set peices from https://en.wikipedia.org/wiki/Conway%27s_Game_of_Life
// Used by seed to setup the game
//
struct shape {
  int   width, height;
  char  shape[];
};

const shape                           // Seed shape definitions
  GLIDER = {3, 3,       // Gliders
    " 1 "    
    "  1"
    "111"},

  GLIDER2 = {3, 3,       
    " 1 "    
    "1  "
    "111"},

  GLIDER3 = {3, 3,
    "111"    
    "1  "
    " 1 "},
  GLIDER4 = {3, 3,
    "111"    
    "  1"
    " 1 "},

  LWSS = {5, 4,         // Light weight space ship
    "1  1 "
    "    1"
    "1   1"
    " 1111"},

  MWSS = {6, 5,         // Middle weight space ship
    "11111 "
    "1    1"
    "1     "
    " 1   1"
    "   1  "},

  HWSS = {7, 5,         // Heavy weight space ship
    " 111111"
    "1     1"
    "      1"
    "1    1 "
    "  11   "},

  RPENT = {3, 3,       // R-Pentomino
    " 11"
    "11 "
    " 1 "},

  DIEHARD = {8, 3,     // Diehard
    "       1"
    "11      "
    " 1   111"},

  ACORN = {7, 3,       // Acorn
    " 1     "
    "   1   "
    "11  111"};

static const shape *seeds[] = {       // Shape selection list, frequency adjusted for more even random selection
  &GLIDER,
  &GLIDER2,
  &GLIDER3,
  &GLIDER4,
  &LWSS,
  &MWSS,
  &HWSS,
  &RPENT,
  &RPENT,
  &DIEHARD,
  &DIEHARD,
  &ACORN,
  &ACORN
};
static const int seedsC = sizeof(seeds)/sizeof(seeds[0]);



uint32_t prevTime = 0;                // Used for frames-per-second throttle

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------
//
void err(int x) {
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);       // Using onboard LED
  for(i=1;;i++) {                     // Loop forever...
    digitalWrite(LED_BUILTIN, i & 1); // LED on/off blink to alert user
    delay(x);
  }
}

void setup(void) {
  Serial.begin(115200);
//  while (!Serial) delay(10);

  ProtomatterStatus status = matrix.begin();
  if(status != PROTOMATTER_OK) {
    Serial.printf("Protomatter begin() status: %d\n", status);
    err(500);                         // Slow blink = Protomatter error
  }

  if (!accel.begin(0x19)) {
    Serial.println("Couldn't find accelerometer");
    err(250);                         // Fast bink = I2C error
  }
  accel.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  matrix.fillScreen(BLACK);
  matrix.setTextColor(DARK_GREEN, BLACK);

  randomSeed(analogRead(0));
}



// MAIN LOOP - RUNS ONCE PER FRAME OF ANIMATION ----------------------------
//
void loop() {
  uint32_t t;                                   // Limit the animation frame rate to MAX_FPS.
  while(((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  //Serial.printf("\n\n\n Cycle \n\n\n");
  int lc = 0;
  for(int y=0; y <HEIGHT; y++) {                // process each cell
    for(int x=0; x <WIDTH; x++) {
      // Serial.printf(" now/next %d %d xy %d %d ", now, next, x, y);

      int s = state[now][x][y];                 // current state

      int l, r, u, d;                           // wrap arround neighbour coords
      l = (x==0)        ? WIDTH-1  : x-1;
      r = (x==WIDTH-1)  ? 0        : x+1;
      u = (y==0)        ? HEIGHT-1 : y-1;
      d = (y==HEIGHT-1) ? 0        : y+1;

      int n = rules[state[now][l][u]].alive + rules[state[now][x][u]].alive + rules[state[now][r][u]].alive
            + rules[state[now][l][y]].alive                                 + rules[state[now][r][y]].alive
            + rules[state[now][l][d]].alive + rules[state[now][x][d]].alive + rules[state[now][r][d]].alive;

      int ns = rules[s].rule[n];                // new state based on current state and alive neighbours
      lc += rules[ns].alive;                    // activity accumulator

      // Serial.printf(" s %d lrud %d %d %d %d n %d ns %d lc %d\n", s, l, r, u, d, n, ns, lc);

      state[next][x][y] = ns;                   // next state update
      matrix.drawPixel(x, y, rules[ns].colour); // display update
    }
  }

  if (millis() < 10000) {                       // Banner message for a few seconds
    banner();
  }
  matrix.show();

  now ^= 1;                                     // swap now/next 0<->1
  next ^= 1;

  if(lc < HEIGHT * WIDTH /25) {                 // Check for low activity and reseed
    seed(1);
  }
  // Serial.println("done");
}

//  Opening banner
//
void banner() {
    matrix.setCursor(1, 1);
    matrix.print("Game of Life");
    matrix.setCursor(1, 9);
    matrix.print("B36/S23");
  
    matrix.setCursor(WIDTH-18*6, HEIGHT-8);  
    matrix.print("R.Lincoln May 2025");
}

//  Seed with n objects
//
void seed(int n) {

  for(int i=0; i<n; i++) {
    int px = random(10, WIDTH -10);             // random screen position
    int py = random(10, HEIGHT -10);

    int s = random(seedsC);                     // random seed matrix
    const shape *seed = seeds[s];

    for(int y=0, i=0; y <seed->height; y++) {   // copy any 1s onto the matrix
      for(int x=0; x <seed->width; x++, i++) {
        if(seed->shape[i] == '1') {
          state[now][px+x][py+y] = 2;
        }
      }
    }
  }

}
