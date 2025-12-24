import serial
import keyboard
import time

PORT = "COM3"     # change this (Arduino IDE shows the port)
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

print("Controls:")
print("  LEFT/RIGHT arrows = move left/right")
print("  UP arrow (hold)   = oscillate back and forth")
print("  P                 = double oscillation speed")
print("  A                 = read angle")
print("  ESC               = stop and quit")
print()

left_was_pressed = False
right_was_pressed = False
up_was_pressed = False
p_was_pressed = False
a_was_pressed = False
up_is_held = False

while True:
    left_pressed = keyboard.is_pressed("left")
    right_pressed = keyboard.is_pressed("right")
    up_pressed = keyboard.is_pressed("up")
    p_pressed = keyboard.is_pressed("p")
    a_pressed = keyboard.is_pressed("a")
    
    # Send L only on key down (not while held)
    if left_pressed and not left_was_pressed:
        ser.write(b"L")
    
    # Send R only on key down (not while held)
    if right_pressed and not right_was_pressed:
        ser.write(b"R")
    
    # UP arrow: start oscillation when pressed, stop when released
    if up_pressed and not up_was_pressed:
        ser.write(b"O")
        up_is_held = True
    
    if not up_pressed and up_was_pressed and up_is_held:
        ser.write(b"S")
        up_is_held = False
    
    # P key: double speed (only on key down)
    if p_pressed and not p_was_pressed:
        ser.write(b"P")
    
    # A key: request angle (only on key down)
    if a_pressed and not a_was_pressed:
        ser.write(b"A")
    
    left_was_pressed = left_pressed
    right_was_pressed = right_pressed
    up_was_pressed = up_pressed
    p_was_pressed = p_pressed
    a_was_pressed = a_pressed
    
    # Read and print any responses from Arduino
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
    
    if keyboard.is_pressed("esc"):
        ser.write(b"S")
        break
    
    time.sleep(0.01)  # Small delay to prevent CPU spinning
