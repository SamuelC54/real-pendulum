# real-pendulum

A real pendulum control system using Arduino and keyboard input from a laptop.

## Setup

### Arduino

1. Open `arduino-code/arduino-code.ino` in Arduino IDE
2. Select your board and port (Tools → Board / Port)
3. Upload the sketch to your Arduino

### Laptop Controller

1. Install Python dependencies:
   ```bash
   pip install pyserial keyboard
   ```

2. Open `laptop-controller/controller.py` and update the `PORT` variable to match your Arduino's COM port (visible in Arduino IDE → Tools → Port)

3. Run the controller:
   ```bash
   python laptop-controller/controller.py
   ```

## Usage

- **LEFT arrow** → sends `L` to Arduino
- **RIGHT arrow** → sends `R` to Arduino
- **ESC** → sends `S` to Arduino and quits the program
