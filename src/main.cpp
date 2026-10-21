/*
 * ARCADE 40x4: TEMPLE RUN + SNAKE + FLAPPY BIRD + TETRIS + FROGGER
 * Arduino Uno + Optrex DMC40457 (40x4 character LCD, two HD44780 controllers)
 * + analog joystick module.
 *
 * Wiring (same as the LiquidCrystal(12, 11, 5, 4, 3, 2) / E2 = 10 sketch)
 *   LCD RS  -> D12 (PB4)          LCD R/W -> GND
 *   LCD E1  -> D11 (PB3)  top two lines (controller 1)
 *   LCD E2  -> D10 (PB2)  bottom two lines (controller 2)
 *   LCD DB4 -> D5  (PD5)          LCD DB5 -> D4 (PD4)
 *   LCD DB6 -> D3  (PD3)          LCD DB7 -> D2 (PD2)
 *   LCD DB0..DB3 -> unconnected
 *   Joystick VRy -> A1   (up / down)
 *   Joystick VRx -> A0   (left / right: menu, Snake steering, Temple Run slide)
 *   Speed pot    -> A2   (wiper; outer legs to 5V and GND, swap them to reverse)
 *   Button       -> D8 (PB0) to GND (menu select, start, Temple Run jump)
 *
 * Home screen
 *   Stick up/down picks a game, stick sideways or the button opens it.
 *   On a game's title or game-over screen: up (or button) plays, sideways
 *   goes back to the home screen.
 *
 * Temple Run
 *   You run along one of 4 lanes. The temple scrolls toward you.
 *     Wall    - solid, change lanes to dodge
 *     Barrier - low fence, jump over it          (ENABLE_JUMP)
 *     Branch  - overhead branch, slide under it  (ENABLE_SLIDE)
 *     Coin    - +10 points
 *   Speed rises every LEVEL_DIST steps, scaled live by the speed pot
 *   (50%..200%). Distance points scale with the pot, so running faster scores
 *   more.
 *
 * Snake
 *   Each character cell is split into a top and bottom half, giving a 34x8
 *   field. Eat food to grow; walls and your own body kill. Food is worth
 *   10 points at 100% speed (scaled by the pot) and every bite speeds you up.
 *
 * Flappy Bird
 *   Push the stick up (or press the button) to flap. The bird moves with
 *   pixel precision: its two character cells are redrawn in CGRAM every
 *   frame. Pipe gaps are placed on half rows and narrow after a while.
 *   Each pipe passed is worth 10 points at 100% speed; the pot sets how fast
 *   the pipes scroll.
 *
 * Tetris (sideways)
 *   The well runs the length of the screen: 34 columns long, 8 half rows
 *   across. Pieces enter at the right and fall left; a full column clears.
 *   Stick up/down moves, right rotates, left soft drops, button hard drops.
 *   40/100/300/1200 points x level for 1-4 lines, scaled by the pot, which
 *   also sets how fast pieces fall. Level rises every 10 lines.
 *
 * Frogger (sideways)
 *   Hop from the grass on the left to the four lily pads on the right edge.
 *   Each column is a lane: cars drive up and down the roads, logs float up
 *   and down the rivers. Ride the logs (don't get carried off the screen),
 *   never touch the water or a car. Stick hops in any direction, the button
 *   hops forward. Fill all four pads to reach the next, faster level. Three
 *   frogs, 40 seconds each; the pot sets how fast the lanes move.
 *
 * Best scores for every game are kept in EEPROM.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
#define ENABLE_JUMP  1
#define ENABLE_SLIDE 1
#define JOY_X_INVERT 0          // set to 1 if left and right come out swapped

#define RS_PIN   (1 << PORTB4)  // D12
#define E1_PIN   (1 << PORTB3)  // D11
#define E2_PIN   (1 << PORTB2)  // D10
#define BTN_PIN  (1 << PORTB0)  // D8 (only used with ENABLE_JUMP)
#define E_BOTH   (E1_PIN | E2_PIN)

#define DATA_PORT PORTD
#define DATA_DDR  DDRD
#define DATA_MASK 0x3C          // PD2..PD5
#define CTRL_PORT PORTB

#define JOY_X_CH  0             // A0
#define JOY_Y_CH  1             // A1
#define JOY_HI    700
#define JOY_LO    300
#define POT_CH    2             // A2

// HD44780 needs ~37 us per instruction/data byte. LiquidCrystal waits 100 us
// per nibble; 80 us per byte keeps a safe margin for this older panel.
#define LCD_EXEC_US 80

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------
#define LCD_W      40
#define LCD_H      4
#define PLAY_W     34           // columns 0..33 are the playfield
#define SEP_COL    34           // divider
#define HUD_COL    35           // columns 35..39 are the HUD
#define PLAYER_COL 4

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------
#define STEP_MS_START  170      // scroll period at level 1
#define STEP_MS_MIN    60
#define STEP_MS_DEC    8        // per level
#define SPEED_MIN_PCT  50       // pot fully one way: half speed
#define SPEED_MAX_PCT  200      // pot fully the other way: double speed
#define STEP_MS_FLOOR  30       // never scroll faster than this
#define SPEED_SHOW_MS  1500     // HUD shows the speed this long after a change
#define LEVEL_DIST     120      // steps per level
#define MAX_LEVEL      15
#define JUMP_STEPS     3
#define SLIDE_STEPS    3
#define LANE_REPEAT_MS 130      // auto-repeat while the stick is held
#define DEBOUNCE_MS    25
#define DEATH_MS       900      // death animation before the restart prompt
#define COIN_POINTS    10
#define SCORE_MAX      99999UL

#define SN_START_LEN     4
#define SN_STEP_MS_START 200    // move period at 100% speed
#define SN_STEP_MS_MIN   80
#define SN_STEP_MS_DEC   3      // per food eaten

#define FL_BIRD_COL      6
#define FL_BIRD_H        4      // sprite height in pixels
#define FL_TICK_MS       20     // physics tick (not scaled by the pot)
#define FL_GRAVITY       6      // 1/64 px per tick^2
#define FL_FLAP          68     // upward speed after a flap, 1/64 px per tick
#define FL_VMAX          96     // terminal fall speed
#define FL_SCROLL_MS     150    // pipe scroll period at 100% speed
#define FL_PIPE_SPACING  9      // columns from one pipe to the next
#define FL_RUNWAY        16     // empty columns before the first pipe
#define FL_GAP_START     4      // gap height in half rows
#define FL_GAP_MIN       3
#define FL_GAP_SHRINK_AT 15     // pipes passed before the gap narrows
#define FL_GAP_SHIFT     3      // max gap move between pipes, half rows

#define TT_FALL_MS_START 400    // gravity period at level 1, 100% speed
#define TT_FALL_MS_MIN   60
#define TT_FALL_MS_DEC   30     // per level
#define TT_LINES_LEVEL   10     // lines per level
#define TT_SOFT_MS       35     // fall period while soft dropping
#define TT_DAS_MS        170    // hold before up/down auto-repeats
#define TT_ARR_MS        70     // up/down auto-repeat period
#define TT_CLEAR_MS      300    // full lines flash this long before vanishing

#define FR_LIVES         3
#define FR_TIME_S        40     // seconds per frog
#define FR_SPLAT_MS      700    // a lost frog shows this long before the next one
#define FR_HOP_REPEAT_MS 180    // hop auto-repeat while the stick is held
#define FR_LANE_MS_START 1200   // lane step period at level 1, 100% speed
#define FR_LANE_MS_MIN   500
#define FR_LANE_MS_DEC   60     // per level
#define FR_HOP_POINTS    10     // each new column reached, per frog
#define FR_HOME_POINTS   50
#define FR_TIME_POINTS   2      // per second left on reaching a pad
#define FR_LEVEL_POINTS  200    // all four pads filled

// ---------------------------------------------------------------------------
// Games
// ---------------------------------------------------------------------------
enum { GAME_TEMPLE, GAME_SNAKE, GAME_FLAPPY, GAME_TETRIS, GAME_FROGGER, NUM_GAMES };

static const char nameTemple[]  PROGMEM = "TEMPLE RUN";
static const char nameSnake[]   PROGMEM = "SNAKE";
static const char nameFlappy[]  PROGMEM = "FLAPPY BIRD";
static const char nameTetris[]  PROGMEM = "TETRIS";
static const char nameFrogger[] PROGMEM = "FROGGER";
static PGM_P const gameNames[NUM_GAMES] PROGMEM = { nameTemple, nameSnake, nameFlappy, nameTetris, nameFrogger };

// ---------------------------------------------------------------------------
// Custom glyphs (CGRAM 0..7, loaded into both controllers per game)
// ---------------------------------------------------------------------------
enum {
  G_RUN1 = 0, G_RUN2, G_JUMP, G_SLIDE, G_BARRIER, G_BRANCH, G_WALL, G_COIN
};
#define G_EMPTY ' '
#define G_DEAD  'X'

static const uint8_t glyphs[8][8] PROGMEM = {
  {0x06, 0x06, 0x04, 0x0E, 0x15, 0x04, 0x0A, 0x11}, // RUN1: stride
  {0x06, 0x06, 0x04, 0x0E, 0x0E, 0x04, 0x0C, 0x04}, // RUN2: passing step
  {0x06, 0x06, 0x15, 0x0E, 0x04, 0x1B, 0x00, 0x00}, // JUMP: tucked, feet up
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0x14}, // SLIDE: flat on ground
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x0A, 0x0A}, // BARRIER: low fence
  {0x1F, 0x1F, 0x04, 0x0A, 0x00, 0x00, 0x00, 0x00}, // BRANCH: overhead
  {0x1F, 0x1B, 0x1B, 0x1F, 0x0E, 0x0E, 0x1F, 0x1B}, // WALL: stone bricks
  {0x00, 0x0E, 0x11, 0x15, 0x15, 0x11, 0x0E, 0x00}, // COIN
};

// Half-cell glyphs (Snake, Tetris): every combination of (top half, bottom
// half) contents, where each half is empty or one of two kinds of block.
// 3*3 - 1 (both empty is a space) = 8.
enum { SC_EMPTY, SC_BODY, SC_FOOD };      // Snake halves
enum { TC_EMPTY, TC_LOCKED, TC_ACTIVE };  // Tetris halves
#define HALF_GLYPH(top, bot) ((top) * 3 + (bot) - 1)

enum { HK_SNAKE, HK_TETRIS };
static const uint8_t halfPatterns[2][2][4] PROGMEM = {
  { {0x1F, 0x1F, 0x1F, 0x00}, {0x04, 0x0E, 0x04, 0x00} },  // body, food
  { {0x1F, 0x1F, 0x1F, 0x00}, {0x1F, 0x11, 0x1F, 0x00} },  // locked, falling
};

static uint8_t half_glyph_row(uint8_t kind, uint8_t g, uint8_t row) {
  uint8_t code = g + 1;
  uint8_t half = (row < 4) ? code / 3 : code % 3;
  return half ? pgm_read_byte(&halfPatterns[kind][half - 1][row & 3]) : 0x00;
}

// Flappy glyphs: pipes have a 3-px body and a full-width lip at the gap.
// The two bird slots are rewritten every frame, merged with whatever pipe
// is behind the bird; the icon slot is a static bird for the title screen.
enum {
  F_PIPE = 0, F_PIPE_END_DN, F_PIPE_END_UP, F_PIPE_TOP, F_PIPE_BOT,
  F_BIRD_HI, F_BIRD_LO, F_BIRD_ICON
};

static const uint8_t flGlyphs[5][8] PROGMEM = {
  {0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E}, // PIPE: body
  {0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F}, // END_DN: upper pipe ends
  {0x1F, 0x1F, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E}, // END_UP: lower pipe starts
  {0x0E, 0x0E, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00}, // TOP: upper pipe ends mid-cell
  {0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x0E, 0x0E}, // BOT: lower pipe starts mid-cell
};

static const uint8_t birdSprites[2][FL_BIRD_H] PROGMEM = {
  {0x16, 0x1D, 0x1E, 0x0C},          // wing up
  {0x06, 0x1D, 0x1E, 0x1C},          // wing down
};

static uint8_t flappy_glyph_row(uint8_t g, uint8_t row) {
  if (g < F_BIRD_HI) return pgm_read_byte(&flGlyphs[g][row]);
  if (g == F_BIRD_ICON && row >= 2 && row < 2 + FL_BIRD_H) return pgm_read_byte(&birdSprites[0][row - 2]);
  return 0x00;
}

// Frogger glyphs: lanes run vertically, so cars are drawn nose up or down.
enum { FG_FROG = 0, FG_CAR_UP, FG_CAR_DN, FG_LOG, FG_WATER, FG_PAD, FG_GRASS, FG_COUNT };

static const uint8_t frGlyphs[FG_COUNT][8] PROGMEM = {
  {0x11, 0x0E, 0x0D, 0x1F, 0x1F, 0x0D, 0x0E, 0x11}, // FROG: facing right
  {0x0A, 0x1F, 0x11, 0x1F, 0x1F, 0x1F, 0x1B, 0x00}, // CAR_UP: windscreen at the top
  {0x00, 0x1B, 0x1F, 0x1F, 0x1F, 0x11, 0x1F, 0x0A}, // CAR_DN
  {0x0E, 0x0E, 0x0A, 0x0E, 0x0E, 0x0E, 0x0A, 0x0E}, // LOG: bark, joins up vertically
  {0x00, 0x08, 0x15, 0x02, 0x00, 0x08, 0x15, 0x02}, // WATER: ripples
  {0x00, 0x0E, 0x1D, 0x1B, 0x1F, 0x1F, 0x0E, 0x00}, // PAD: empty lily pad
  {0x00, 0x0A, 0x04, 0x00, 0x00, 0x0A, 0x04, 0x00}, // GRASS: tufts
};

enum { GS_TEMPLE, GS_SNAKE, GS_FLAPPY, GS_TETRIS, GS_FROGGER, GS_NONE = 0xFF };

static uint8_t glyph_row(uint8_t set, uint8_t g, uint8_t row) {
  switch (set) {
    case GS_TEMPLE:  return pgm_read_byte(&glyphs[g][row]);
    case GS_SNAKE:   return half_glyph_row(HK_SNAKE, g, row);
    case GS_TETRIS:  return half_glyph_row(HK_TETRIS, g, row);
    case GS_FROGGER: return (g < FG_COUNT) ? pgm_read_byte(&frGlyphs[g][row]) : 0x00;
    default:        return flappy_glyph_row(g, row);
  }
}

// ---------------------------------------------------------------------------
// Timebase: Timer2 CTC @ 1 kHz (Timer0 is left alone for the Arduino core)
// ---------------------------------------------------------------------------
static volatile uint32_t g_ms;

ISR(TIMER2_COMPA_vect) { g_ms++; }

static uint32_t now_ms() {
  uint32_t t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = g_ms; }
  return t;
}

// ---------------------------------------------------------------------------
// RNG: xorshift16 (smaller and faster than rand())
// ---------------------------------------------------------------------------
static uint16_t rng = 0xACE1;

static uint8_t rnd(uint8_t n) {  // 0..n-1
  rng ^= rng << 7;
  rng ^= rng >> 9;
  rng ^= rng << 8;
  return ((uint16_t)(uint8_t)(rng >> 8) * n) >> 8;
}

// ---------------------------------------------------------------------------
// LCD driver (4-bit, write-only, dual controller)
// ---------------------------------------------------------------------------
static uint8_t shown[LCD_H][LCD_W];  // what is on the glass
static uint8_t frame[LCD_H][LCD_W];  // what we want on the glass
static uint8_t cursorAddr[2];        // tracked DDRAM address per controller
static uint8_t glyphSet = GS_NONE;   // which glyphs are in CGRAM

// LCD DB4..DB7 sit on PD5, PD4, PD3, PD2 (reversed order), so each 4-bit
// value is translated through a lookup table instead of a plain shift.
static const uint8_t nibbleToPort[16] = {
  0x00, 0x20, 0x10, 0x30, 0x08, 0x28, 0x18, 0x38,
  0x04, 0x24, 0x14, 0x34, 0x0C, 0x2C, 0x1C, 0x3C,
};

// Sends the upper four bits of v.
static void lcd_nibble(uint8_t v, uint8_t e) {
  DATA_PORT = (DATA_PORT & ~DATA_MASK) | nibbleToPort[v >> 4];
  CTRL_PORT |= e;
  _delay_us(1);                      // E pulse >= 450 ns
  CTRL_PORT &= ~e;
  _delay_us(1);
}

// Clock one byte into a controller without waiting for it to execute.
static void lcd_send(uint8_t v, uint8_t e, uint8_t rs) {
  if (rs) CTRL_PORT |= RS_PIN;
  else    CTRL_PORT &= ~RS_PIN;
  lcd_nibble(v, e);
  lcd_nibble(v << 4, e);
}

static void lcd_write(uint8_t v, uint8_t e, uint8_t rs) {
  lcd_send(v, e, rs);
  _delay_us(LCD_EXEC_US);
}

static inline void lcd_cmd(uint8_t c, uint8_t e) { lcd_write(c, e, 0); }

// Both controllers are initialised in lock-step by strobing E1 and E2 together.
static void lcd_init() {
  CTRL_PORT &= ~(RS_PIN | E_BOTH);
  _delay_ms(50);                     // power-on settle

  lcd_nibble(0x30, E_BOTH); _delay_ms(5);
  lcd_nibble(0x30, E_BOTH); _delay_us(150);
  lcd_nibble(0x30, E_BOTH); _delay_us(150);
  lcd_nibble(0x20, E_BOTH); _delay_us(150);   // now in 4-bit mode

  lcd_cmd(0x28, E_BOTH);             // 4-bit, 2 lines, 5x8
  lcd_cmd(0x08, E_BOTH);             // display off
  lcd_cmd(0x01, E_BOTH); _delay_ms(2);
  lcd_cmd(0x06, E_BOTH);             // increment, no shift
  lcd_cmd(0x0C, E_BOTH);             // display on, cursor off

  cursorAddr[0] = cursorAddr[1] = 0;
  memset(shown, ' ', sizeof(shown));
}

// Each game has its own 8 custom characters; swap them in when it opens.
static void load_glyphs(uint8_t set) {
  if (set == glyphSet) return;
  glyphSet = set;
  lcd_cmd(0x40, E_BOTH);             // CGRAM address 0
  for (uint8_t g = 0; g < 8; g++) {
    for (uint8_t row = 0; row < 8; row++) lcd_write(glyph_row(set, g, row), E_BOTH, 1);
  }
  lcd_cmd(0x80, E_BOTH);             // back to DDRAM 0
  cursorAddr[0] = cursorAddr[1] = 0;
}

// Rewrite consecutive CGRAM slots starting at slot g (8 bytes each).
static void lcd_set_glyphs(uint8_t g, const uint8_t *rows, uint8_t len) {
  lcd_cmd(0x40 | (g << 3), E_BOTH);
  for (uint8_t i = 0; i < len; i++) lcd_write(rows[i], E_BOTH, 1);
  lcd_cmd(0x80, E_BOTH);
  cursorAddr[0] = cursorAddr[1] = 0;
}

// Each controller owns 80 cells (its two rows). They are scanned playfield
// first (row A cols 0..33, row B cols 0..33), then the divider + HUD, so the
// moving objects always land before the score digits.
#define CELLS_PER_CTL (2 * LCD_W)
#define SIDE_W        (LCD_W - PLAY_W)

// Produce the next byte controller k needs: a set-address command when the
// next changed cell isn't where its cursor already is, otherwise the
// character itself (riding auto-increment). Returns false when k is done.
static bool next_op(uint8_t k, uint8_t &pos, uint8_t &val, uint8_t &rs) {
  while (pos < CELLS_PER_CTL) {
    uint8_t sub, c;
    if (pos < 2 * PLAY_W) {
      sub = pos >= PLAY_W;
      c   = pos - (sub ? PLAY_W : 0);
    } else {
      uint8_t p = pos - 2 * PLAY_W;
      sub = p >= SIDE_W;
      c   = PLAY_W + p - (sub ? SIDE_W : 0);
    }
    const uint8_t r = 2 * k + sub;
    if (frame[r][c] != shown[r][c]) {
      const uint8_t addr = (sub ? 0x40 : 0x00) + c;
      if (cursorAddr[k] != addr) {
        val = 0x80 | addr; rs = 0;
        cursorAddr[k] = addr;
        return true;                 // same cell again next call, for its data
      }
      val = frame[r][c]; rs = 1;
      shown[r][c] = val;
      cursorAddr[k] = addr + 1;
      pos++;
      return true;
    }
    pos++;
  }
  return false;
}

// Push only the cells that changed, to both controllers in parallel: while
// one is busy executing a byte the other is being fed, so a full-screen
// update costs about half the time and the top and bottom halves move together.
static void lcd_flush() {
  uint8_t pos0 = 0, pos1 = 0, v, rs;
  for (;;) {
    bool a = next_op(0, pos0, v, rs);
    if (a) lcd_send(v, E1_PIN, rs);
    bool b = next_op(1, pos1, v, rs);
    if (b) lcd_send(v, E2_PIN, rs);
    if (!a && !b) break;
    _delay_us(LCD_EXEC_US);          // each controller got at most one byte
  }
}

// ---------------------------------------------------------------------------
// Frame buffer helpers
// ---------------------------------------------------------------------------
static void fb_clear() { memset(frame, ' ', sizeof(frame)); }

static void fb_fill(uint8_t r, uint8_t c, uint8_t len, uint8_t ch) {
  memset(&frame[r][c], ch, len);
}

static void fb_text_P(uint8_t r, uint8_t c, PGM_P s) {
  uint8_t ch;
  while (c < LCD_W && (ch = pgm_read_byte(s++))) frame[r][c++] = ch;
}

static void fb_center_P(uint8_t r, uint8_t c, uint8_t width, PGM_P s) {
  uint8_t len = strlen_P(s);
  fb_text_P(r, c + ((len < width) ? (width - len) / 2 : 0), s);
}

// Right-aligned, space-padded decimal.
static void fb_num(uint8_t r, uint8_t c, uint32_t v, uint8_t width) {
  for (int8_t i = width - 1; i >= 0; i--) {
    frame[r][c + i] = (v || i == width - 1) ? '0' + (v % 10) : ' ';
    v /= 10;
  }
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------
static uint16_t adc_read(uint8_t ch) {
  ADMUX = (1 << REFS0) | ch;         // AVcc reference
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// Returns true once per debounced press.
static bool button_pressed(uint32_t now) {
#if ENABLE_JUMP
  static bool     btnState = true;   // treat a held button at boot as already pressed
  static uint32_t btnChangeMs;
  bool raw = !(PINB & BTN_PIN);
  if (raw != btnState && now - btnChangeMs >= DEBOUNCE_MS) {
    btnState = raw;
    btnChangeMs = now;
    return raw;
  }
#else
  (void)now;
#endif
  return false;
}

static uint16_t potFilt;
static uint8_t  speedPct = 100;
static uint32_t speedChangedMs;

// Smoothed pot reading with a little hysteresis so the value doesn't flicker.
static void read_speed(uint32_t now) {
  potFilt = (potFilt * 3 + adc_read(POT_CH)) >> 2;
  uint8_t pct = SPEED_MIN_PCT + (uint32_t)potFilt * (SPEED_MAX_PCT - SPEED_MIN_PCT) / 1023;
  if (pct > speedPct + 1 || pct + 1 < speedPct) {
    speedPct = pct;
    speedChangedMs = now;
  }
}

// Stick directions double as Snake headings (clockwise from up).
enum { S_UP, S_RIGHT, S_DOWN, S_LEFT, S_NONE };

static const int8_t dirDx[4] = { 0, 1, 0, -1 };
static const int8_t dirDy[4] = { -1, 0, 1, 0 };

// Dominant stick direction, or S_NONE when centred.
static uint8_t stick_dir() {
  int16_t y = (int16_t)adc_read(JOY_Y_CH) - 512;
  int16_t x = (int16_t)adc_read(JOY_X_CH) - 512;
#if JOY_X_INVERT
  x = -x;
#endif
  int16_t ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
  if (ay >= ax) {
    if (y > JOY_HI - 512) return S_UP;
    if (y < JOY_LO - 512) return S_DOWN;
  } else {
    if (x > JOY_HI - 512) return S_RIGHT;
    if (x < JOY_LO - 512) return S_LEFT;
  }
  return S_NONE;
}

static uint8_t stickLast = S_UP;     // treat a stick held at boot as already pushed

// A fresh push out of the centre, once; S_NONE otherwise.
static uint8_t stick_edge() {
  uint8_t d = stick_dir();
  uint8_t e = (stickLast == S_NONE) ? d : S_NONE;
  stickLast = d;
  return e;
}

// The stick must return to centre before stick_edge() fires again.
static void stick_lock() { stickLast = S_UP; }

// ---------------------------------------------------------------------------
// Persistent best scores (one slot per game; slot 0 is the original layout)
// ---------------------------------------------------------------------------
#define EE_MAGIC     0x7E52
#define EE_SLOT_SIZE 6               // magic word + best dword

static uint32_t load_best(uint8_t g) {
  uint16_t base = g * EE_SLOT_SIZE;
  if (eeprom_read_word((uint16_t *)base) != EE_MAGIC) return 0;
  uint32_t b = eeprom_read_dword((uint32_t *)(base + 2));
  return (b > SCORE_MAX) ? 0 : b;
}

static void save_best(uint8_t g, uint32_t b) {
  uint16_t base = g * EE_SLOT_SIZE;
  eeprom_update_word((uint16_t *)base, EE_MAGIC);
  eeprom_update_dword((uint32_t *)(base + 2), b);
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------
enum State : uint8_t {
  ST_MENU, ST_TITLE, ST_PLAY, ST_DEAD, SN_TITLE, SN_PLAY, SN_DEAD, FL_TITLE, FL_PLAY, FL_DEAD,
  TT_TITLE, TT_PLAY, TT_DEAD, FR_TITLE, FR_PLAY, FR_DEAD
};

static const State deadStates[NUM_GAMES] = { ST_DEAD, SN_DEAD, FL_DEAD, TT_DEAD, FR_DEAD };

static State    state = ST_MENU;
static uint8_t  game;                // selected / running game
static uint32_t score, bests[NUM_GAMES];
static bool     newBest;
static uint32_t deathMs;

static void die(uint32_t now) {
  state = deadStates[game];
  deathMs = now;
  stick_lock();                      // must release the stick before retrying
  if (score > bests[game]) {
    bests[game] = score;
    newBest = true;
    save_best(game, score);
  }
}

// Step period for a base period, scaled by the speed pot.
static uint16_t scaled_period(uint16_t baseMs) {
  uint16_t period = (uint16_t)baseMs * 100 / speedPct;
  return (period < STEP_MS_FLOOR) ? STEP_MS_FLOOR : period;
}

// Fixed cadence: schedule from the ideal tick, not from "now", so loop
// latency doesn't accumulate into uneven steps. If we fell a whole period
// behind, resync instead of bursting.
static bool step_due(uint32_t now, uint32_t &last, uint16_t period) {
  if (now - last < period) return false;
  last += period;
  if (now - last >= period) last = now;
  return true;
}

// ---------------------------------------------------------------------------
// Temple Run
// ---------------------------------------------------------------------------
static uint8_t  world[LCD_H][PLAY_W];
static uint8_t  lane;
static uint8_t  airTicks, slideTicks;
static uint8_t  animFrame;
static uint8_t  level;
static uint8_t  stepMs;
static uint8_t  gap;
static uint8_t  coinLane, coinLeft;
static uint32_t distance, coins;
static uint32_t runPoints;           // distance points in hundredths, weighted by speed
static uint32_t lastStepMs, lastMoveMs;
static int8_t   lastDir;
#define DIR_LOCKED 2
static bool     slideHeld;

static void start_game(uint32_t now) {
  memset(world, G_EMPTY, sizeof(world));
  lane = 1;
  airTicks = slideTicks = 0;
  animFrame = 0;
  level = 1;
  stepMs = STEP_MS_START;
  gap = 10;                          // a short runway before the first obstacle
  coinLeft = 0;
  distance = coins = score = runPoints = 0;
  newBest = false;
  lastStepMs = now;
  lastDir = DIR_LOCKED;              // the start push must not also move a lane
  slideHeld = true;                 // ignore a stick already pushed at start
  state = ST_PLAY;
}

static void update_score() {
  score = runPoints / 100 + coins * COIN_POINTS;
  if (score > SCORE_MAX) score = SCORE_MAX;
}

static void check_collision(uint32_t now) {
  uint8_t &cell = world[lane][PLAYER_COL];
  switch (cell) {
    case G_COIN:    coins++; cell = G_EMPTY; update_score(); break;
    case G_WALL:    die(now); break;
    case G_BARRIER: if (!airTicks)   die(now); break;
    case G_BRANCH:  if (!slideTicks) die(now); break;
  }
}

// Build the column entering on the right edge. At most two walls per
// obstacle column, so there is always a lane you can survive in.
static void spawn_column(uint8_t col[LCD_H]) {
  for (uint8_t r = 0; r < LCD_H; r++) col[r] = G_EMPTY;

  if (gap) {
    gap--;
  } else {
    const uint8_t wallP    = 18 + level * 2;
    const uint8_t barrierP = ENABLE_JUMP ? wallP + 22 : wallP;
    const uint8_t branchP  = ENABLE_SLIDE ? barrierP + 18 : barrierP;
    uint8_t walls = 0, placed = 0;
    uint8_t start = rnd(LCD_H);
    for (uint8_t i = 0; i < LCD_H; i++) {
      uint8_t r = (start + i) & 3;
      uint8_t p = rnd(100);
      if (p < wallP && walls < 2)  { col[r] = G_WALL; walls++; placed++; }
      else if (p < barrierP)       { col[r] = G_BARRIER; placed++; }
      else if (p < branchP)        { col[r] = G_BRANCH;  placed++; }
    }
    if (!placed) col[rnd(LCD_H)] = G_WALL;

    uint8_t minGap = (level < 9) ? 8 - level / 2 : 4;
    gap = minGap + rnd(4);
  }

  // Coin trails weave through whatever lane is free.
  if (coinLeft) {
    if (col[coinLane] == G_EMPTY) col[coinLane] = G_COIN;
    coinLeft--;
  } else if (rnd(100) < 7) {
    coinLane = rnd(LCD_H);
    coinLeft = 3 + rnd(6);
  }
}

static void game_step(uint32_t now) {
  for (uint8_t r = 0; r < LCD_H; r++) memmove(&world[r][0], &world[r][1], PLAY_W - 1);

  uint8_t col[LCD_H];
  spawn_column(col);
  for (uint8_t r = 0; r < LCD_H; r++) world[r][PLAY_W - 1] = col[r];

  distance++;
  runPoints += speedPct;
  animFrame ^= 1;
  if (distance % LEVEL_DIST == 0 && level < MAX_LEVEL) {
    level++;
    uint8_t dec = (level - 1) * STEP_MS_DEC;
    stepMs = (STEP_MS_START - STEP_MS_MIN > dec) ? STEP_MS_START - dec : STEP_MS_MIN;
  }
  update_score();

  check_collision(now);
  if (airTicks)   airTicks--;
  if (slideTicks) slideTicks--;
}

static void handle_input(uint32_t now, bool pressed) {
  // Lanes: edge-triggered with auto-repeat, never blocks the game loop.
  uint16_t y = adc_read(JOY_Y_CH);
  int8_t dir = (y > JOY_HI) ? -1 : (y < JOY_LO) ? 1 : 0;
  if (dir == 0) {
    lastDir = 0;
  } else if (lastDir == DIR_LOCKED) {
    // wait for the stick to return to centre
  } else if (dir != lastDir || now - lastMoveMs >= LANE_REPEAT_MS) {
    int8_t target = (int8_t)lane + dir;
    if (target >= 0 && target < LCD_H) {
      lane = target;
      check_collision(now);
    }
    lastDir = dir;
    lastMoveMs = now;
  }
  if (state != ST_PLAY) return;

  if (pressed && !airTicks && !slideTicks) airTicks = JUMP_STEPS;

#if ENABLE_SLIDE
  // Either sideways direction slides, so stick orientation doesn't matter.
  uint16_t x = adc_read(JOY_X_CH);
  bool slide = (x > JOY_HI) || (x < JOY_LO);
  if (slide && !slideHeld && !airTicks && !slideTicks) slideTicks = SLIDE_STEPS;
  slideHeld = slide;
#endif
}

// ---------------------------------------------------------------------------
// Snake (34x8 field: two half-height cells per character)
// ---------------------------------------------------------------------------
#define SN_W     PLAY_W
#define SN_H     (LCD_H * 2)
#define SN_CELLS (SN_W * SN_H)

// The body is stored as the heading of each link from tail to head, 2 bits
// each, in a ring buffer; snOcc has one bit per row for fast collision tests.
static uint8_t  snOcc[SN_W];
static uint8_t  snLinks[SN_CELLS / 4];
static uint16_t snTailIdx, snHeadIdx;
static uint8_t  snHeadX, snHeadY, snTailX, snTailY, snFoodX, snFoodY;
static uint8_t  snDir, snNext;
static uint16_t snLen;
static uint8_t  snStepMs;
static uint32_t snLastStepMs;
static bool     snArmed;

static uint8_t sn_get_link(uint16_t i) {
  return (snLinks[i >> 2] >> ((i & 3) * 2)) & 3;
}

static void sn_set_link(uint16_t i, uint8_t d) {
  uint8_t sh = (i & 3) * 2;
  snLinks[i >> 2] = (snLinks[i >> 2] & ~(3 << sh)) | (d << sh);
}

static uint16_t sn_next_idx(uint16_t i) { return (i + 1 == SN_CELLS) ? 0 : i + 1; }

static bool sn_occupied(uint8_t x, uint8_t y) { return (snOcc[x] >> y) & 1; }

// Food goes on a random free cell; false when the board is full.
static bool sn_place_food() {
  uint8_t x = rnd(SN_W), y = rnd(SN_H);
  for (uint16_t i = 0; i < SN_CELLS; i++) {
    if (!sn_occupied(x, y)) {
      snFoodX = x;
      snFoodY = y;
      return true;
    }
    if (++x == SN_W) {
      x = 0;
      if (++y == SN_H) y = 0;
    }
  }
  return false;
}

static void sn_start(uint32_t now) {
  memset(snOcc, 0, sizeof(snOcc));
  snTailX = 2;
  snTailY = snHeadY = SN_H / 2 - 1;
  snHeadX = snTailX + SN_START_LEN - 1;
  for (uint8_t x = snTailX; x <= snHeadX; x++) snOcc[x] |= 1 << snTailY;
  for (uint8_t i = 0; i < SN_START_LEN - 1; i++) sn_set_link(i, S_RIGHT);
  snTailIdx = 0;
  snHeadIdx = SN_START_LEN - 1;
  snLen = SN_START_LEN;
  snDir = snNext = S_RIGHT;
  snArmed = false;                   // ignore the start push until the stick centres
  snStepMs = SN_STEP_MS_START;
  score = 0;
  newBest = false;
  sn_place_food();
  snLastStepMs = now;
  state = SN_PLAY;
}

static void sn_input() {
  uint8_t d = stick_dir();
  if (d == S_NONE) snArmed = true;
  else if (snArmed && d != ((snDir + 2) & 3)) snNext = d;  // no reversing into yourself
}

static void sn_step(uint32_t now) {
  snDir = snNext;
  int8_t nx = (int8_t)snHeadX + dirDx[snDir];
  int8_t ny = (int8_t)snHeadY + dirDy[snDir];
  if (nx < 0 || nx >= SN_W || ny < 0 || ny >= SN_H) { die(now); return; }

  bool eat = (nx == snFoodX && ny == snFoodY);
  if (!eat) {
    // The tail leaves first, so following right behind it is legal.
    snOcc[snTailX] &= ~(1 << snTailY);
    uint8_t d = sn_get_link(snTailIdx);
    snTailX += dirDx[d];
    snTailY += dirDy[d];
    snTailIdx = sn_next_idx(snTailIdx);
  }
  if (sn_occupied(nx, ny)) { die(now); return; }

  sn_set_link(snHeadIdx, snDir);
  snHeadIdx = sn_next_idx(snHeadIdx);
  snHeadX = nx;
  snHeadY = ny;
  snOcc[nx] |= 1 << ny;

  if (eat) {
    snLen++;
    score += speedPct / 10;
    if (score > SCORE_MAX) score = SCORE_MAX;
    if (snStepMs >= SN_STEP_MS_MIN + SN_STEP_MS_DEC) snStepMs -= SN_STEP_MS_DEC;
    if (!sn_place_food()) die(now);  // board full: nothing left to eat
  }
}

// ---------------------------------------------------------------------------
// Flappy Bird (pipes on half rows, bird with pixel precision)
// ---------------------------------------------------------------------------
#define FL_Y_MAX ((LCD_H * 8 - FL_BIRD_H) * 64)  // bird top at the ground, 1/64 px

static uint8_t  flCols[PLAY_W];      // per column, bit h = half row h is pipe
static int16_t  flY, flVy;           // bird top and speed, 1/64 px
static uint8_t  flSpawn, flLastGap;
static uint16_t flPipes;
static uint32_t flLastTickMs, flLastScrollMs;
static uint8_t  flBirdRows[16];      // what is in the two bird CGRAM slots
static bool     flBirdValid;

static void fl_start(uint32_t now) {
  memset(flCols, 0, sizeof(flCols));
  flY = (LCD_H * 8 - FL_BIRD_H) / 2 * 64;
  flVy = -FL_FLAP;                   // the start push is the first flap
  flSpawn = FL_RUNWAY;
  flLastGap = (8 - FL_GAP_START) / 2;
  flPipes = 0;
  score = 0;
  newBest = false;
  flLastTickMs = flLastScrollMs = now;
  state = FL_PLAY;
}

// True if the bird's pixel rows overlap a pipe in its column.
static bool fl_hit() {
  uint8_t m = flCols[FL_BIRD_COL];
  if (!m) return false;
  uint8_t yp = flY >> 6;
  for (uint8_t i = 0; i < FL_BIRD_H; i++) {
    if ((m >> ((yp + i) >> 2)) & 1) return true;
  }
  return false;
}

static void fl_tick(uint32_t now) {
  flVy += FL_GRAVITY;
  if (flVy > FL_VMAX) flVy = FL_VMAX;
  flY += flVy;
  if (flY < 0) {                     // the ceiling just stops you
    flY = 0;
    flVy = 0;
  }
  if (flY >= FL_Y_MAX) {
    flY = FL_Y_MAX;
    die(now);
    return;
  }
  if (fl_hit()) die(now);
}

// New pipe column with its gap at most FL_GAP_SHIFT half rows from the last.
static uint8_t fl_spawn_column() {
  if (flSpawn) {
    flSpawn--;
    return 0;
  }
  flSpawn = FL_PIPE_SPACING - 1;
  uint8_t gapH = (flPipes < FL_GAP_SHRINK_AT) ? FL_GAP_START : FL_GAP_MIN;
  uint8_t lo = (flLastGap > FL_GAP_SHIFT) ? flLastGap - FL_GAP_SHIFT : 0;
  uint8_t hi = flLastGap + FL_GAP_SHIFT;
  if (hi > 8 - gapH) hi = 8 - gapH;
  flLastGap = lo + rnd(hi - lo + 1);
  return (uint8_t)~(((1 << gapH) - 1) << flLastGap);
}

static void fl_scroll(uint32_t now) {
  memmove(&flCols[0], &flCols[1], PLAY_W - 1);
  flCols[PLAY_W - 1] = fl_spawn_column();
  if (fl_hit()) {
    die(now);
    return;
  }
  if (flCols[FL_BIRD_COL - 1]) {     // a pipe just went past the bird
    flPipes++;
    score += speedPct / 10;
    if (score > SCORE_MAX) score = SCORE_MAX;
  }
}

// Glyph for character row r of a pipe column with half-row mask m.
static uint8_t fl_pipe_glyph(uint8_t m, uint8_t r) {
  bool top = (m >> (2 * r)) & 1, bot = (m >> (2 * r + 1)) & 1;
  if (top && bot) {
    if (r < LCD_H - 1 && !((m >> (2 * r + 2)) & 1)) return F_PIPE_END_DN;
    if (r > 0 && !((m >> (2 * r - 1)) & 1))         return F_PIPE_END_UP;
    return F_PIPE;
  }
  if (top) return F_PIPE_TOP;
  if (bot) return F_PIPE_BOT;
  return ' ';
}

// Compose the bird sprite over the pipe behind it into the two bird slots,
// uploading only when the pixels change. Returns the pixel offset in the cell.
static uint8_t fl_render_bird(uint8_t spriteFrame) {
  uint8_t yp = flY >> 6, rB = yp >> 3, o = yp & 7;
  uint8_t rows[16];
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t g = (rB + k < LCD_H) ? fl_pipe_glyph(flCols[FL_BIRD_COL], rB + k) : ' ';
    for (uint8_t i = 0; i < 8; i++) rows[k * 8 + i] = (g == ' ') ? 0 : pgm_read_byte(&flGlyphs[g][i]);
  }
  for (uint8_t i = 0; i < FL_BIRD_H; i++) rows[o + i] |= pgm_read_byte(&birdSprites[spriteFrame][i]);

  if (!flBirdValid || memcmp(rows, flBirdRows, sizeof(rows))) {
    lcd_set_glyphs(F_BIRD_HI, rows, sizeof(rows));
    memcpy(flBirdRows, rows, sizeof(rows));
    flBirdValid = true;
  }
  return o;
}

// ---------------------------------------------------------------------------
// Tetris, played the long way: the well is 34 columns long and 8 half rows
// across. Pieces enter at the right and fall left; column 0 is the floor, so
// a cleared line is a full column. Each column is a bitmask of its 8 halves.
// ---------------------------------------------------------------------------
#define TT_LEN   PLAY_W
#define TT_WIDE  (LCD_H * 2)

// 4x4 piece bitmaps, bit 15 = (row 0, col 0). A piece row maps to the
// distance from the floor (row 0 is farthest), a piece column to the half row.
static const uint16_t ttShapes[7][4] PROGMEM = {
  {0x0F00, 0x2222, 0x00F0, 0x4444},  // I
  {0x44C0, 0x8E00, 0x6440, 0x0E20},  // J
  {0x4460, 0x0E80, 0xC440, 0x2E00},  // L
  {0xCC00, 0xCC00, 0xCC00, 0xCC00},  // O
  {0x06C0, 0x8C40, 0x6C00, 0x4620},  // S
  {0x0E40, 0x4C40, 0x4E00, 0x4640},  // T
  {0x0C60, 0x4C80, 0xC600, 0x2640},  // Z
};

static const uint16_t ttLineScore[5] = { 0, 40, 100, 300, 1200 };

static uint8_t  ttWell[TT_LEN];
static uint8_t  ttPiece, ttRot, ttNext;
static int8_t   ttX, ttY;            // piece origin: distance from floor, half row
static uint8_t  ttLevel;
static uint16_t ttLines;
static uint8_t  ttStick;             // stick direction last loop
static bool     ttSoftOk;            // soft drop needs a fresh push per piece
static bool     ttClearing;
static uint32_t ttLastFallMs, ttRepeatMs, ttClearMs;

// Cell i (0..3) of a piece; returns false once all four are listed.
static bool tt_cell(uint8_t p, uint8_t r, int8_t px, int8_t py, uint8_t i, int8_t &x, int8_t &y) {
  uint16_t s = pgm_read_word(&ttShapes[p][r]);
  for (uint8_t b = 0; b < 16; b++) {
    if (!(s & (0x8000u >> b))) continue;
    if (i-- == 0) {
      x = px + 3 - (b >> 2);
      y = py + (b & 3);
      return true;
    }
  }
  return false;
}

static bool tt_fits(uint8_t p, uint8_t r, int8_t px, int8_t py) {
  int8_t x, y;
  for (uint8_t i = 0; tt_cell(p, r, px, py, i, x, y); i++) {
    if (x < 0 || x >= TT_LEN || y < 0 || y >= TT_WIDE) return false;
    if ((ttWell[x] >> y) & 1) return false;
  }
  return true;
}

static uint16_t tt_fall_ms() {
  uint16_t dec = (uint16_t)(ttLevel - 1) * TT_FALL_MS_DEC;
  return (TT_FALL_MS_START - TT_FALL_MS_MIN > dec) ? TT_FALL_MS_START - dec : TT_FALL_MS_MIN;
}

static void tt_spawn(uint32_t now) {
  ttPiece = ttNext;
  ttNext = rnd(7);
  ttRot = 0;
  ttX = TT_LEN - 4;
  ttY = (TT_WIDE - 4) / 2;
  ttSoftOk = (ttStick != S_LEFT);
  ttLastFallMs = now;
  if (!tt_fits(ttPiece, ttRot, ttX, ttY)) die(now);  // stacked out
}

static void tt_start(uint32_t now) {
  memset(ttWell, 0, sizeof(ttWell));
  ttLevel = 1;
  ttLines = 0;
  ttClearing = false;
  ttStick = S_UP;                    // the start push must not also move the piece
  score = 0;
  newBest = false;
  ttNext = rnd(7);
  state = TT_PLAY;
  tt_spawn(now);
}

static void tt_lock(uint32_t now) {
  int8_t x, y;
  for (uint8_t i = 0; tt_cell(ttPiece, ttRot, ttX, ttY, i, x, y); i++) ttWell[x] |= 1 << y;
  for (uint8_t c = 0; c < TT_LEN; c++) {
    if (ttWell[c] == 0xFF) {         // flash full lines before removing them
      ttClearing = true;
      ttClearMs = now;
      return;
    }
  }
  tt_spawn(now);
}

static void tt_clear_lines(uint32_t now) {
  uint8_t n = 0, dst = 0;
  for (uint8_t src = 0; src < TT_LEN; src++) {
    if (ttWell[src] == 0xFF) n++;
    else                     ttWell[dst++] = ttWell[src];
  }
  while (dst < TT_LEN) ttWell[dst++] = 0;

  score += (uint32_t)ttLineScore[n] * ttLevel * speedPct / 100;
  if (score > SCORE_MAX) score = SCORE_MAX;
  ttLines += n;
  ttLevel = 1 + ttLines / TT_LINES_LEVEL;
  ttClearing = false;
  tt_spawn(now);
}

// Gravity: one step toward the floor, locking if it can't move.
static void tt_fall(uint32_t now) {
  if (tt_fits(ttPiece, ttRot, ttX - 1, ttY)) ttX--;
  else                                       tt_lock(now);
}

// Rotate with simple kicks: try shifting across, then away from the floor.
static void tt_rotate() {
  static const int8_t kicks[][2] = { {0, 0}, {0, -1}, {0, 1}, {0, -2}, {0, 2}, {1, 0} };
  uint8_t r = (ttRot + 1) & 3;
  for (uint8_t k = 0; k < sizeof(kicks) / sizeof(kicks[0]); k++) {
    if (tt_fits(ttPiece, r, ttX + kicks[k][0], ttY + kicks[k][1])) {
      ttRot = r;
      ttX += kicks[k][0];
      ttY += kicks[k][1];
      return;
    }
  }
}

// Stick up/down moves across the well (with auto-repeat), right rotates,
// left soft drops, the button hard drops.
static void tt_input(uint32_t now, bool pressed) {
  uint8_t d = stick_dir();
  bool fresh = (d != ttStick);
  if (fresh) {
    ttStick = d;
    ttRepeatMs = now + TT_DAS_MS;
    ttSoftOk = true;
  }
  if (ttClearing) return;

  if (d == S_UP || d == S_DOWN) {
    if (fresh || (int32_t)(now - ttRepeatMs) >= 0) {
      if (!fresh) ttRepeatMs = now + TT_ARR_MS;
      int8_t ny = ttY + ((d == S_UP) ? -1 : 1);
      if (tt_fits(ttPiece, ttRot, ttX, ny)) ttY = ny;
    }
  } else if (d == S_RIGHT && fresh) {
    tt_rotate();
  }

  if (pressed) {
    while (tt_fits(ttPiece, ttRot, ttX - 1, ttY)) ttX--;
    tt_lock(now);
  }
}

static bool tt_soft_dropping() { return ttSoftOk && ttStick == S_LEFT; }

// ---------------------------------------------------------------------------
// Frogger, played left to right: every playfield column is a lane whose
// traffic moves up or down. Each moving lane is a 16-position ring of which
// rows 0..3 are on screen, so cars and logs drift in and out of view.
// ---------------------------------------------------------------------------
#define FR_RING  16

// S = grass (safe), R = road, W = river, H = the four lily pads.
static const char frLayout[PLAY_W + 1] PROGMEM =
  "S" "RRRRRRR" "S" "WWWWWWW" "S" "RRRRRRRR" "S" "WWWWWWW" "H";

static uint16_t frRing[PLAY_W];      // bit r = car / log at ring position r
static int16_t  frSpd[PLAY_W];       // step period in ms, sign = direction (+ is down)
static uint32_t frLast[PLAY_W];
static uint8_t  frCol, frRow, frBestCol;
static uint8_t  frLives, frLevel, frHomes;
static uint8_t  frStick;
static bool     frSplat;
static uint32_t frLifeMs, frSplatMs, frRepeatMs;

static uint8_t fr_lane_type(uint8_t c) { return pgm_read_byte(&frLayout[c]); }

static void fr_add(uint16_t points) {
  score += (uint32_t)points * speedPct / 100;
  if (score > SCORE_MAX) score = SCORE_MAX;
}

// Random ring: cars are 1-2 long with wide gaps, logs 1-4 long with short
// gaps. Both get denser as the level rises.
static uint16_t fr_make_lane(bool water) {
  uint16_t m = 0;
  uint8_t pos = rnd(3);
  while (pos < FR_RING) {
    uint8_t len, gapLen;
    if (water) {
      len    = ((frLevel > 3) ? 1 : 2) + rnd(3);
      gapLen = 1 + rnd(2) + (frLevel > 2);
    } else {
      len    = rnd(4) ? 1 : 2;
      gapLen = ((frLevel < 5) ? 5 - frLevel / 2 : 3) + rnd(3);
    }
    for (uint8_t i = 0; i < len && pos < FR_RING; i++) m |= 1u << pos++;
    pos += gapLen;
  }
  return m;
}

static void fr_setup_lanes(uint32_t now) {
  uint16_t dec  = (uint16_t)(frLevel - 1) * FR_LANE_MS_DEC;
  uint16_t base = (FR_LANE_MS_START - FR_LANE_MS_MIN > dec) ? FR_LANE_MS_START - dec : FR_LANE_MS_MIN;
  uint8_t flip = rnd(2);
  for (uint8_t c = 0; c < PLAY_W; c++) {
    uint8_t t = fr_lane_type(c);
    if (t != 'R' && t != 'W') {
      frRing[c] = 0;
      frSpd[c] = 0;
      continue;
    }
    frRing[c] = fr_make_lane(t == 'W');
    int16_t period = base + rnd(base / 20) * 10;  // 1x..1.5x the base period
    frSpd[c] = ((c + flip) & 1) ? period : -period;  // neighbouring lanes go opposite ways
    frLast[c] = now;
  }
}

static void fr_respawn(uint32_t now) {
  frCol = 0;
  frRow = 1;
  frBestCol = 0;
  frSplat = false;
  frLifeMs = now;
}

static void fr_start(uint32_t now) {
  frLevel = 1;
  frLives = FR_LIVES;
  frHomes = 0;
  frStick = S_UP;                    // the start push must not also hop
  score = 0;
  newBest = false;
  fr_setup_lanes(now);
  fr_respawn(now);
  state = FR_PLAY;
}

static uint8_t fr_time_left(uint32_t now) {
  uint32_t el = now - frLifeMs, limit = FR_TIME_S * 1000UL;
  return (el >= limit) ? 0 : (limit - el + 999) / 1000;
}

// Lose the current frog; the game ends with the last one.
static void fr_splat(uint32_t now) {
  if (--frLives == 0) {
    die(now);
    return;
  }
  frSplat = true;
  frSplatMs = now;
}

static void fr_reach_pad(uint32_t now) {
  frHomes |= 1 << frRow;
  fr_add(FR_HOME_POINTS + fr_time_left(now) * FR_TIME_POINTS);
  if (frHomes == 0x0F) {
    frHomes = 0;
    frLevel++;
    fr_add(FR_LEVEL_POINTS);
    fr_setup_lanes(now);
  }
  fr_respawn(now);
}

// Whatever is under the frog decides its fate after a hop.
static void fr_check(uint32_t now) {
  bool on = (frRing[frCol] >> frRow) & 1;
  switch (fr_lane_type(frCol)) {
    case 'R': if (on) fr_splat(now); break;
    case 'W': if (!on) fr_splat(now); break;
    case 'H':
      if ((frHomes >> frRow) & 1) fr_splat(now);  // pad already taken
      else                        fr_reach_pad(now);
      break;
  }
}

static void fr_hop(uint32_t now, uint8_t d) {
  int8_t nc = (int8_t)frCol + dirDx[d];
  int8_t nr = (int8_t)frRow + dirDy[d];
  if (nc < 0 || nc >= PLAY_W || nr < 0 || nr >= LCD_H) return;
  frCol = nc;
  frRow = nr;
  if (frCol > frBestCol) {
    frBestCol = frCol;
    fr_add(FR_HOP_POINTS);
  }
  fr_check(now);
}

// One hop per push with auto-repeat while held; the button hops forward.
static void fr_input(uint32_t now, bool pressed) {
  uint8_t d = stick_dir();
  bool fresh = (d != frStick);
  if (fresh) {
    frStick = d;
    frRepeatMs = now + FR_HOP_REPEAT_MS;
  }
  if (frSplat) return;

  if (d != S_NONE && (fresh || (int32_t)(now - frRepeatMs) >= 0)) {
    if (!fresh) frRepeatMs = now + FR_HOP_REPEAT_MS;
    fr_hop(now, d);
  }
  if (pressed && state == FR_PLAY && !frSplat) fr_hop(now, S_RIGHT);
}

// Move every lane that is due. A log carries the frog along; a car that
// arrives on the frog's cell squashes it.
static void fr_lanes(uint32_t now) {
  for (uint8_t c = 0; c < PLAY_W; c++) {
    int16_t s = frSpd[c];
    if (!s) continue;
    if (!step_due(now, frLast[c], scaled_period((s < 0) ? -s : s))) continue;
    uint16_t m = frRing[c];
    frRing[c] = (s > 0) ? (m << 1) | (m >> (FR_RING - 1)) : (m >> 1) | (m << (FR_RING - 1));

    if (c != frCol || frSplat) continue;
    if (fr_lane_type(c) == 'W') {
      int8_t r = (int8_t)frRow + ((s > 0) ? 1 : -1);
      if (r < 0 || r >= LCD_H) fr_splat(now);   // carried off the screen
      else                     frRow = r;
    } else if ((frRing[c] >> frRow) & 1) {
      fr_splat(now);
    }
    if (state != FR_PLAY) return;
  }
}

static void fr_update(uint32_t now) {
  if (frSplat) {
    if (now - frSplatMs >= FR_SPLAT_MS) fr_respawn(now);
  } else if (!fr_time_left(now)) {
    fr_splat(now);
  }
  if (state == FR_PLAY) fr_lanes(now);
}

// ---------------------------------------------------------------------------
// Screens
// ---------------------------------------------------------------------------
static void draw_menu(uint32_t now) {
  fb_clear();
  if ((now >> 11) & 1) fb_center_P(0, 0, LCD_W, PSTR("U/D: CHOOSE    L/R OR BUTTON: PLAY"));
  else                 fb_center_P(0, 0, LCD_W, PSTR("====  SELECT A GAME  ===="));
  // Three rows of games; the list scrolls to keep the selection visible.
  static uint8_t top;
  if (game < top)     top = game;
  if (game > top + 2) top = game - 2;
  for (uint8_t i = top; i < NUM_GAMES && i < top + 3; i++) {
    uint8_t r = 1 + i - top;
    if (i == game) {
      frame[r][3] = '>';
      if ((now >> 8) & 1) frame[r][4] = '>';
    }
    fb_text_P(r, 6, (PGM_P)pgm_read_word(&gameNames[i]));
    fb_text_P(r, 24, PSTR("BEST"));
    fb_num(r, 29, bests[i], 5);
  }
  if (top > 0)             frame[1][37] = '^';
  if (top + 3 < NUM_GAMES) frame[3][37] = 'v';
}

// Bottom row shared by both title screens.
static void draw_title_footer(uint32_t now, PGM_P go) {
  fb_text_P(3, 1, PSTR("KNOB SPEED:"));
  fb_num(3, 12, speedPct, 3);
  frame[3][15] = '%';
  if ((now >> 9) & 1) fb_text_P(3, 21, go);
  else                fb_text_P(3, 21, PSTR("L/R: BACK TO MENU"));
}

static void draw_play(uint32_t now) {
  for (uint8_t r = 0; r < LCD_H; r++) {
    memcpy(frame[r], world[r], PLAY_W);
    frame[r][SEP_COL] = '|';
  }

  uint8_t g;
  if (state == ST_DEAD)  g = ((now - deathMs) & 0x80) ? G_DEAD : G_RUN1;
  else if (airTicks)     g = G_JUMP;
  else if (slideTicks)   g = G_SLIDE;
  else                   g = animFrame ? G_RUN2 : G_RUN1;
  if (state == ST_DEAD && now - deathMs >= DEATH_MS) g = G_DEAD;
  frame[lane][PLAYER_COL] = g;

  fb_text_P(0, HUD_COL, PSTR("SCORE"));
  fb_num(1, HUD_COL, score, 5);
  frame[2][HUD_COL] = G_COIN;
  fb_num(2, HUD_COL + 1, coins, 4);
  if (now - speedChangedMs < SPEED_SHOW_MS) {
    fb_num(3, HUD_COL, speedPct, 4);
    frame[3][HUD_COL + 4] = '%';
  } else {
    fb_text_P(3, HUD_COL, PSTR("LV"));
    fb_num(3, HUD_COL + 2, level, 3);
  }
}

#define OV_COL 5
#define OV_W   24

static void draw_game_over(uint32_t now) {
  for (uint8_t r = 1; r < LCD_H; r++) fb_fill(r, OV_COL, OV_W, ' ');
  fb_center_P(1, OV_COL, OV_W, PSTR("GAME  OVER!"));
  fb_text_P(2, OV_COL + 1, PSTR("SCORE"));
  fb_num(2, OV_COL + 7, score, 5);
  fb_text_P(2, OV_COL + 13, PSTR("BEST"));
  fb_num(2, OV_COL + 18, bests[game], 5);
  if ((now >> 9) & 1)  fb_center_P(3, OV_COL, OV_W, PSTR("UP: AGAIN   L/R: MENU"));
  else if (newBest)    fb_center_P(3, OV_COL, OV_W, PSTR("** NEW BEST! **"));
}

static void draw_title(uint32_t now) {
  fb_clear();
  frame[0][3] = G_RUN1;
  fb_center_P(0, 0, LCD_W, PSTR("== TEMPLE  RUN  40x4 =="));
  frame[0][36] = G_COIN;
  fb_text_P(1, 1, PSTR("STICK U/D: lane"));
#if ENABLE_JUMP
  fb_text_P(2, 1, PSTR("BUTTON   : jump"));
#endif
#if ENABLE_SLIDE
  fb_text_P(1, 21, PSTR("STICK L/R: slide"));
#endif
  fb_text_P(2, 21, PSTR("BEST:"));
  fb_num(2, 27, bests[GAME_TEMPLE], 5);
  draw_title_footer(now, PSTR("PUSH UP TO RUN >>"));
}

static void draw_snake(uint32_t now) {
  // While dying the body blinks; the food stays put.
  bool body = !(state == SN_DEAD && now - deathMs < DEATH_MS && ((now - deathMs) & 0x80));
  for (uint8_t r = 0; r < LCD_H; r++) {
    for (uint8_t c = 0; c < SN_W; c++) {
      uint8_t half[2];
      for (uint8_t h = 0; h < 2; h++) {
        uint8_t y = 2 * r + h;
        if (body && sn_occupied(c, y))          half[h] = SC_BODY;
        else if (c == snFoodX && y == snFoodY)  half[h] = SC_FOOD;
        else                                    half[h] = SC_EMPTY;
      }
      frame[r][c] = (half[0] || half[1]) ? HALF_GLYPH(half[0], half[1]) : ' ';
    }
    frame[r][SEP_COL] = '|';
  }

  fb_text_P(0, HUD_COL, PSTR("SCORE"));
  fb_num(1, HUD_COL, score, 5);
  fb_text_P(2, HUD_COL, PSTR("LN"));
  fb_num(2, HUD_COL + 2, snLen, 3);
  fb_num(3, HUD_COL, speedPct, 4);
  frame[3][HUD_COL + 4] = '%';
}

static void draw_snake_title(uint32_t now) {
  fb_clear();
  fb_fill(0, 2, 5, HALF_GLYPH(SC_BODY, SC_EMPTY));
  frame[0][8] = HALF_GLYPH(SC_FOOD, SC_EMPTY);
  fb_center_P(0, 0, LCD_W, PSTR("~~~  S N A K E  ~~~"));
  frame[0][31] = HALF_GLYPH(SC_FOOD, SC_EMPTY);
  fb_fill(0, 33, 5, HALF_GLYPH(SC_BODY, SC_EMPTY));
  fb_text_P(1, 1, PSTR("STICK: steer"));
  fb_text_P(1, 21, PSTR("EAT FOOD TO GROW"));
  fb_text_P(2, 1, PSTR("AVOID WALLS & TAIL"));
  fb_text_P(2, 21, PSTR("BEST:"));
  fb_num(2, 27, bests[GAME_SNAKE], 5);
  draw_title_footer(now, PSTR("PUSH UP TO START"));
}

static void draw_flappy(uint32_t now) {
  for (uint8_t r = 0; r < LCD_H; r++) {
    for (uint8_t c = 0; c < PLAY_W; c++) frame[r][c] = fl_pipe_glyph(flCols[c], r);
    frame[r][SEP_COL] = '|';
  }

  // While dying the bird blinks.
  bool bird = !(state == FL_DEAD && now - deathMs < DEATH_MS && ((now - deathMs) & 0x80));
  if (bird) {
    uint8_t o = fl_render_bird((state == FL_PLAY) ? (now >> 7) & 1 : 0);
    uint8_t rB = (flY >> 6) >> 3;
    frame[rB][FL_BIRD_COL] = F_BIRD_HI;
    if (o > 8 - FL_BIRD_H) frame[rB + 1][FL_BIRD_COL] = F_BIRD_LO;
  }

  fb_text_P(0, HUD_COL, PSTR("SCORE"));
  fb_num(1, HUD_COL, score, 5);
  if (now - speedChangedMs < SPEED_SHOW_MS) {
    fb_text_P(2, HUD_COL, PSTR("SPEED"));
    fb_num(3, HUD_COL, speedPct, 4);
    frame[3][HUD_COL + 4] = '%';
  } else {
    fb_text_P(2, HUD_COL, PSTR("PIPES"));
    fb_num(3, HUD_COL, flPipes, 5);
  }
}

static void draw_flappy_title(uint32_t now) {
  fb_clear();
  frame[0][3] = F_BIRD_ICON;
  fb_center_P(0, 0, LCD_W, PSTR("~~  FLAPPY  BIRD  ~~"));
  frame[0][34] = F_PIPE_TOP;
  frame[0][36] = F_PIPE_TOP;
  fb_text_P(1, 1, PSTR("UP/BUTTON: flap"));
  fb_text_P(1, 21, PSTR("FLY THRU THE PIPES"));
  fb_text_P(2, 1, PSTR("DON'T HIT GROUND"));
  fb_text_P(2, 21, PSTR("BEST:"));
  fb_num(2, 27, bests[GAME_FLAPPY], 5);
  draw_title_footer(now, PSTR("PUSH UP TO FLY"));
}

static void draw_tetris(uint32_t now) {
  // Falling piece as a half-row mask per column.
  uint8_t act[TT_LEN];
  memset(act, 0, sizeof(act));
  bool dying = (state == TT_DEAD && now - deathMs < DEATH_MS && ((now - deathMs) & 0x80));
  if (!ttClearing && !dying) {
    int8_t x, y;
    for (uint8_t i = 0; tt_cell(ttPiece, ttRot, ttX, ttY, i, x, y); i++) act[x] |= 1 << y;
  }
  bool hideFull = ttClearing && ((now - ttClearMs) & 0x40);

  for (uint8_t r = 0; r < LCD_H; r++) {
    for (uint8_t c = 0; c < TT_LEN; c++) {
      uint8_t well = (dying || (hideFull && ttWell[c] == 0xFF)) ? 0 : ttWell[c];
      uint8_t half[2];
      for (uint8_t h = 0; h < 2; h++) {
        uint8_t y = 2 * r + h;
        if ((act[c] >> y) & 1)       half[h] = TC_ACTIVE;
        else if ((well >> y) & 1)    half[h] = TC_LOCKED;
        else                         half[h] = TC_EMPTY;
      }
      frame[r][c] = (half[0] || half[1]) ? HALF_GLYPH(half[0], half[1]) : ' ';
    }
    frame[r][SEP_COL] = '|';
  }

  // HUD: next piece (rows 0-1, same orientation as the well), score, level.
  fb_fill(0, HUD_COL, SIDE_W - 1, ' ');
  fb_fill(1, HUD_COL, SIDE_W - 1, ' ');
  frame[0][HUD_COL] = 'N';
  frame[1][HUD_COL] = 'X';
  uint8_t next[4] = {0, 0, 0, 0};
  int8_t x, y;
  for (uint8_t i = 0; tt_cell(ttNext, 0, 0, 0, i, x, y); i++) next[x] |= 1 << y;
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t c = 0; c < 4; c++) {
      bool t = (next[c] >> (2 * r)) & 1, b = (next[c] >> (2 * r + 1)) & 1;
      if (t || b) frame[r][HUD_COL + 1 + c] = HALF_GLYPH(t ? TC_ACTIVE : TC_EMPTY, b ? TC_ACTIVE : TC_EMPTY);
    }
  }
  fb_num(2, HUD_COL, score, 5);
  if (now - speedChangedMs < SPEED_SHOW_MS) {
    fb_num(3, HUD_COL, speedPct, 4);
    frame[3][HUD_COL + 4] = '%';
  } else {
    fb_text_P(3, HUD_COL, PSTR("LV"));
    fb_num(3, HUD_COL + 2, ttLevel, 3);
  }
}

static void draw_tetris_title(uint32_t now) {
  fb_clear();
  frame[0][2] = HALF_GLYPH(TC_ACTIVE, TC_ACTIVE);
  frame[0][3] = HALF_GLYPH(TC_ACTIVE, TC_EMPTY);
  fb_center_P(0, 0, LCD_W, PSTR("<<<  T E T R I S  <<<"));
  frame[0][35] = HALF_GLYPH(TC_LOCKED, TC_LOCKED);
  frame[0][36] = HALF_GLYPH(TC_EMPTY, TC_LOCKED);
  fb_text_P(1, 1, PSTR("U/D: move  R: turn"));
  fb_text_P(1, 21, PSTR("BLOCKS FALL LEFT"));
  fb_text_P(2, 1, PSTR("L: drop  BTN: slam"));
  fb_text_P(2, 21, PSTR("BEST:"));
  fb_num(2, 27, bests[GAME_TETRIS], 5);
  draw_title_footer(now, PSTR("PUSH UP TO START"));
}

static uint8_t fr_cell_glyph(uint8_t c, uint8_t r) {
  bool on = (frRing[c] >> r) & 1;
  switch (fr_lane_type(c)) {
    case 'R': return on ? ((frSpd[c] > 0) ? FG_CAR_DN : FG_CAR_UP) : ' ';
    case 'W': return on ? FG_LOG : FG_WATER;
    case 'H': return ((frHomes >> r) & 1) ? FG_FROG : FG_PAD;
    default:  return FG_GRASS;
  }
}

static void draw_frogger(uint32_t now) {
  for (uint8_t r = 0; r < LCD_H; r++) {
    for (uint8_t c = 0; c < PLAY_W; c++) frame[r][c] = fr_cell_glyph(c, r);
    frame[r][SEP_COL] = '|';
  }

  // A lost frog blinks as an X where it died.
  uint8_t g = FG_FROG;
  if (state == FR_DEAD || frSplat) {
    uint32_t since = now - ((state == FR_DEAD) ? deathMs : frSplatMs);
    if ((since & 0x80) || (state == FR_DEAD && since >= DEATH_MS)) g = G_DEAD;
  }
  frame[frRow][frCol] = g;

  fb_text_P(0, HUD_COL, PSTR("SCORE"));
  fb_num(1, HUD_COL, score, 5);
  frame[2][HUD_COL] = FG_FROG;
  frame[2][HUD_COL + 1] = '0' + frLives;
  frame[2][HUD_COL + 2] = ' ';
  fb_num(2, HUD_COL + 3, fr_time_left(now), 2);
  if (now - speedChangedMs < SPEED_SHOW_MS) {
    fb_num(3, HUD_COL, speedPct, 4);
    frame[3][HUD_COL + 4] = '%';
  } else {
    fb_text_P(3, HUD_COL, PSTR("LV"));
    fb_num(3, HUD_COL + 2, frLevel, 3);
  }
}

static void draw_frogger_title(uint32_t now) {
  fb_clear();
  frame[0][3] = FG_FROG;
  fb_center_P(0, 0, LCD_W, PSTR(">>>  F R O G G E R  >>>"));
  frame[0][36] = FG_PAD;
  fb_text_P(1, 1, PSTR("STICK: hop  BTN: >"));
  fb_text_P(1, 21, PSTR("FILL ALL 4 PADS ->"));
  fb_text_P(2, 1, PSTR("RIDE LOGS, NO SWIM"));
  fb_text_P(2, 21, PSTR("BEST:"));
  fb_num(2, 27, bests[GAME_FROGGER], 5);
  draw_title_footer(now, PSTR("PUSH UP TO HOP"));
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------
static void open_game() {
  switch (game) {
    case GAME_SNAKE:
      load_glyphs(GS_SNAKE);
      state = SN_TITLE;
      break;
    case GAME_FLAPPY:
      load_glyphs(GS_FLAPPY);
      flBirdValid = false;
      state = FL_TITLE;
      break;
    case GAME_TETRIS:
      load_glyphs(GS_TETRIS);
      state = TT_TITLE;
      break;
    case GAME_FROGGER:
      load_glyphs(GS_FROGGER);
      state = FR_TITLE;
      break;
    default:
      load_glyphs(GS_TEMPLE);
      state = ST_TITLE;
      break;
  }
}

// Title and game-over screens: up (or the button) plays, sideways goes back
// to the menu. Returns true if the screen was left.
static bool title_input(uint32_t now, bool pressed) {
  uint8_t e = stick_edge();
  if (e == S_UP || pressed) {
    if (game == GAME_SNAKE)       sn_start(now);
    else if (game == GAME_FLAPPY) fl_start(now);
    else if (game == GAME_TETRIS) tt_start(now);
    else if (game == GAME_FROGGER) fr_start(now);
    else                          start_game(now);
    return true;
  }
  if (e == S_LEFT || e == S_RIGHT) {
    state = ST_MENU;
    return true;
  }
  return false;
}

// Stir the RNG with ADC noise and timing until the player commits.
static void stir_rng(uint32_t now) {
  rng ^= adc_read(JOY_X_CH) ^ (uint16_t)now;
  if (!rng) rng = 0xACE1;
}

// ---------------------------------------------------------------------------
// Setup / main loop
// ---------------------------------------------------------------------------
static void init_hardware() {
  DATA_DDR |= DATA_MASK;             // LCD DB4..DB7
  DDRB  |= RS_PIN | E_BOTH;
#if ENABLE_JUMP
  DDRB  &= ~BTN_PIN;
  PORTB |= BTN_PIN;                  // pull-up
#endif

  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // 125 kHz

  TCCR2A = (1 << WGM21);             // CTC
  TCCR2B = (1 << CS22);              // clk/64 -> 250 kHz
  OCR2A  = 249;                      // 1 kHz
  TIMSK2 = (1 << OCIE2A);
  sei();

  lcd_init();
}

int main() {
  init_hardware();
  for (uint8_t g = 0; g < NUM_GAMES; g++) bests[g] = load_best(g);
  potFilt = adc_read(POT_CH);        // start the filter at the real knob position
  read_speed(0);

  for (;;) {
    uint32_t now = now_ms();
    bool pressed = button_pressed(now);
    read_speed(now);

    switch (state) {
      case ST_MENU: {
        stir_rng(now);
        uint8_t e = stick_edge();
        if (e == S_UP && game > 0)                     game--;
        else if (e == S_DOWN && game < NUM_GAMES - 1)  game++;
        if (pressed || e == S_LEFT || e == S_RIGHT) open_game();
        else                                        draw_menu(now);
        break;
      }

      case ST_TITLE:
        stir_rng(now);
        if (!title_input(now, pressed)) draw_title(now);
        break;

      case ST_PLAY:
        handle_input(now, pressed);
        if (state == ST_PLAY && step_due(now, lastStepMs, scaled_period(stepMs))) game_step(now);
        draw_play(now);
        break;

      case ST_DEAD:
        draw_play(now);
        if (now - deathMs >= DEATH_MS) {
          draw_game_over(now);
          title_input(now, pressed);
        }
        break;

      case SN_TITLE:
        stir_rng(now);
        if (!title_input(now, pressed)) draw_snake_title(now);
        break;

      case SN_PLAY:
        sn_input();
        if (step_due(now, snLastStepMs, scaled_period(snStepMs))) sn_step(now);
        draw_snake(now);
        break;

      case SN_DEAD:
        draw_snake(now);
        if (now - deathMs >= DEATH_MS) {
          draw_game_over(now);
          title_input(now, pressed);
        }
        break;

      case FL_TITLE:
        stir_rng(now);
        if (!title_input(now, pressed)) draw_flappy_title(now);
        break;

      case FL_PLAY:
        if (pressed || stick_edge() == S_UP) flVy = -FL_FLAP;
        if (step_due(now, flLastTickMs, FL_TICK_MS)) fl_tick(now);
        if (state == FL_PLAY && step_due(now, flLastScrollMs, scaled_period(FL_SCROLL_MS))) fl_scroll(now);
        draw_flappy(now);
        break;

      case FL_DEAD:
        draw_flappy(now);
        if (now - deathMs >= DEATH_MS) {
          draw_game_over(now);
          title_input(now, pressed);
        }
        break;

      case TT_TITLE:
        stir_rng(now);
        if (!title_input(now, pressed)) draw_tetris_title(now);
        break;

      case TT_PLAY:
        tt_input(now, pressed);
        if (state == TT_PLAY) {
          if (ttClearing) {
            if (now - ttClearMs >= TT_CLEAR_MS) tt_clear_lines(now);
          } else {
            uint16_t period = tt_soft_dropping() ? TT_SOFT_MS : scaled_period(tt_fall_ms());
            if (step_due(now, ttLastFallMs, period)) tt_fall(now);
          }
        }
        draw_tetris(now);
        break;

      case TT_DEAD:
        draw_tetris(now);
        if (now - deathMs >= DEATH_MS) {
          draw_game_over(now);
          title_input(now, pressed);
        }
        break;

      case FR_TITLE:
        stir_rng(now);
        if (!title_input(now, pressed)) draw_frogger_title(now);
        break;

      case FR_PLAY:
        fr_input(now, pressed);
        if (state == FR_PLAY) fr_update(now);
        draw_frogger(now);
        break;

      case FR_DEAD:
        draw_frogger(now);
        if (now - deathMs >= DEATH_MS) {
          draw_game_over(now);
          title_input(now, pressed);
        }
        break;
    }

    lcd_flush();
  }
}
