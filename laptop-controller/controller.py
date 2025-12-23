import serial
import keyboard
import time

PORT = "COM3"     # change this (Arduino IDE shows the port)
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

print("Arrow keys: LEFT/RIGHT. ESC quits.")

left_was_pressed = False
right_was_pressed = False

while True:
    left_pressed = keyboard.is_pressed("left")
    right_pressed = keyboard.is_pressed("right")
    
    # Send L only on key down (not while held)
    if left_pressed and not left_was_pressed:
        ser.write(b"L")
    
    # Send R only on key down (not while held)
    if right_pressed and not right_was_pressed:
        ser.write(b"R")
    
    left_was_pressed = left_pressed
    right_was_pressed = right_pressed
    
    if keyboard.is_pressed("esc"):
        ser.write(b"S")
        break
    
    time.sleep(0.01)  # Small delay to prevent CPU spinning
