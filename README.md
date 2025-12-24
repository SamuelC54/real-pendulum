# real-pendulum

A real pendulum control system using Arduino and keyboard input from a laptop.

## Setup

### Arduino

1. Open `arduino-code/arduino-code.ino` in Arduino IDE
2. Select your board and port (Tools → Board / Port)
3. Upload the sketch to your Arduino

### Arduino CLI (Optional)

Install `arduino-cli` to upload directly from the controller:

1. Download from https://arduino.github.io/arduino-cli/latest/installation/
2. Add to PATH
3. Install the board core:
   ```bash
   arduino-cli core install arduino:avr
   ```

### Laptop Controller

1. Install Python dependencies:
   ```bash
   pip install -r laptop-controller/requirements.txt
   ```

2. Open `laptop-controller/controller.py` and update:
   - `PORT` → your Arduino's COM port (e.g., `COM3`)
   - `BOARD` → your board type (default: `arduino:avr:uno`)

3. Run the controller:
   ```bash
   python laptop-controller/controller.py
   ```

### Development Mode (Hot Reload)

For development, use the hot-reload wrapper that auto-restarts when you edit `controller.py`:

```bash
python laptop-controller/dev.py
```

## Controls

| Key | Action |
|-----|--------|
| **LEFT/RIGHT arrows** | Move cart left/right |
| **UP arrow (hold)** | Oscillate back and forth |
| **P** | Double oscillation speed |
| **A** | Read pendulum angle |
| **U** | Upload Arduino code (via arduino-cli) |
| **ESC** | Stop and quit |
