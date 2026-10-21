# 40x4 LCD Arcade

Five games for an Arduino Uno and a 40x4 character LCD, played with an analog joystick, one button and a speed knob:

- **Temple Run**: dodge, jump and slide through a scrolling temple
- **Snake**: the classic, on a 34x8 half-cell field
- **Flappy Bird**: pixel-smooth flapping through pipes
- **Tetris**: played sideways, pieces fall from right to left
- **Frogger**: hop left to right across roads and rivers

The best score for each game is saved in EEPROM, so it survives power cycles.

## Hardware

| Part | Notes |
|---|---|
| Arduino Uno (ATmega328P) | Other 328P boards at 16 MHz work too |
| Optrex DMC40457 40x4 character LCD | Or any 40x4 LCD with two HD44780 controllers (E1 / E2) |
| Analog joystick module | Two-axis, e.g. KY-023 |
| 10 kΩ potentiometer | Game speed knob |
| Push button | Momentary, normally open |
| Contrast pot for the LCD | Usually 10 kΩ on the LCD's V0 pin |

### Wiring

| From | To Arduino |
|---|---|
| LCD RS | D12 |
| LCD R/W | GND |
| LCD E1 (top two rows) | D11 |
| LCD E2 (bottom two rows) | D10 |
| LCD DB4 | D5 |
| LCD DB5 | D4 |
| LCD DB6 | D3 |
| LCD DB7 | D2 |
| LCD DB0–DB3 | not connected |
| Joystick VRx (left / right) | A0 |
| Joystick VRy (up / down) | A1 |
| Joystick +5V / GND | 5V / GND |
| Speed pot wiper | A2 (outer legs to 5V and GND) |
| Button | D8 to GND (internal pull-up, no resistor needed) |

Power the LCD from 5V and GND, and wire its contrast pin and backlight the way your module's datasheet shows. This is the same wiring as the usual `LiquidCrystal(12, 11, 5, 4, 3, 2)` setup with E2 on pin 10.

## Install

The project builds with [PlatformIO](https://platformio.org/).

### With VS Code

1. Install [VS Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Clone the repo:
   ```sh
   git clone https://github.com/audy910/4x40-LED-Matrix-Arcade.git
   ```
3. In VS Code, choose **File → Open Folder** and open the cloned folder. PlatformIO installs the AVR toolchain the first time, which takes a few minutes.
4. Plug in the Uno over USB.
5. Click the **→ (Upload)** arrow in the blue status bar, or run **PlatformIO: Upload** from the command palette.

### From the command line

With the PlatformIO Core CLI installed (`pip install platformio`):

```sh
git clone https://github.com/audy910/4x40-LED-Matrix-Arcade.git
cd 4x40-LED-Matrix-Arcade
pio run                  # build
pio run -t upload        # build and flash the Uno
```

PlatformIO usually finds the board's serial port on its own. If it doesn't, add `upload_port = COM3` (or `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on macOS) under `[env:uno]` in `platformio.ini`.

## Playing

### Menu

- **Stick up / down**: choose a game
- **Stick left / right or button**: open it

On a game's title or game-over screen, push **up** (or press the button) to play, or push **sideways** to go back to the menu.

### The speed knob

The pot on A2 sets game speed from **50% to 200%**, live, even mid-game. Points scale with speed, so playing faster scores more. The HUD shows the new speed for a moment whenever you turn it.

### Temple Run

You run along one of four lanes while the temple scrolls toward you.

- **Stick up / down**: change lane
- **Button**: jump over low barriers
- **Stick left / right**: slide under overhead branches
- **Walls** are solid, so change lanes to get past them
- **Coins** are worth 10 points each

The run speeds up every level. You also score for distance covered.

### Snake

Each character cell is split into a top and bottom half, which gives a 34x8 field.

- **Stick**: steer
- Eat food to grow; each bite is worth 10 points at 100% speed and speeds you up
- Hitting a wall or your own tail ends the game

### Flappy Bird

- **Stick up or button**: flap
- Fly through the gaps in the pipes, and don't hit the ground
- Each pipe passed is worth 10 points at 100% speed
- The gaps get narrower after 15 pipes

### Tetris (sideways)

The well runs the length of the screen: 34 cells long and 8 half rows across. Pieces enter on the right and fall to the left, and a full column clears.

- **Stick up / down**: move the piece
- **Stick right**: rotate
- **Stick left**: soft drop
- **Button**: hard drop

Clearing 1, 2, 3 or 4 lines at once scores 40, 100, 300 or 1200 points times the level. The level goes up every 10 lines, and pieces fall faster.

### Frogger (sideways)

Hop from the grass on the left to the four lily pads on the right edge. Every column is a lane, and traffic moves up and down.

- **Stick**: hop one cell in any direction (hold to keep hopping)
- **Button**: hop forward (right)
- **Roads**: touching a car costs a frog
- **Rivers**: ride the logs. Falling in the water, or being carried off the top or bottom of the screen, costs a frog
- **Pads**: land on an empty pad to bring that frog home. Landing on a pad that's already full costs a frog

You have 3 frogs and 40 seconds for each one. You score for every new column reached, for each frog brought home (plus a bonus for time left) and for filling all four pads, which also starts a faster, busier level.

## Configuration

Options and tuning values are `#define`s near the top of [src/main.cpp](src/main.cpp):

| Define | What it does |
|---|---|
| `JOY_X_INVERT` | Set to `1` if left and right come out swapped |
| `ENABLE_JUMP` | `0` disables the button and Temple Run jumps |
| `ENABLE_SLIDE` | `0` disables Temple Run slides |
| `JOY_HI` / `JOY_LO` | Joystick dead-zone thresholds |
| `STEP_MS_*`, `SN_*`, `FL_*`, `TT_*`, `FR_*` | Speed and difficulty for each game |

To reverse the speed knob, swap its two outer legs.

## Troubleshooting

- **The screen is blank or shows solid blocks**: adjust the LCD contrast pot first. Then check that E1 and E2 aren't swapped and that R/W is tied to GND.
- **Only half the screen works**: one of the enable lines (D11 or D10) isn't connected.
- **The menu scrolls on its own**: the joystick isn't centred near 512. Check its 5V and GND, or widen `JOY_HI` / `JOY_LO`.
- **Left and right are swapped**: set `JOY_X_INVERT` to `1`.
- **To reset the best scores**: clear the Uno's EEPROM, for example by flashing the Arduino IDE's `EEPROM > eeprom_clear` example and then re-flashing this project.
