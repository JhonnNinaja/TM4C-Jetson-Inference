

# Unidirectional Robot Control with Jetson Infenrence

This project uses Python code on a Jetson Nano and C code on a TM4C1294 microcontroller to control a unidirectional robot. 

## Python Object Detection

The Python code utilizes the Jetson Inference Library to run real-time object detection using the camera:

```python
net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold = 0.5)

while True:
    img = camera.Capture()
    detections = net.Detect(img)

    for detection in detections:
        if net.GetClassDesc(detection.ClassID) == "person":
            print("Person detected")
```

It looks for objects like persons and cats. When detected, it sets flags to trigger the next movement sequence.

## UART Communication 

The Python code sends direction commands over UART serial communication to the microcontroller:

```python
ser = serial.Serial('/dev/ttyACM0', 115200) 

ser.write(b'UP') # Send UP command
ser.write(b'LEFT') # Send LEFT command
```

## C Motor Control

The C code on the TM4C microcontroller receives the UART commands and generates PWM signals to control the motors:

```c
void UARTIntHandler() {

  if(data[0]=='U' && data[1]=='P') {
    
    // Set PWM for UP motion
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, value); 
  }
  
  else if(data[0]=='L' && data[1]=='E'&& data[2]=='F'&& data[3]=='T') {

    // Set PWM for LEFT motion
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, value);
  }
}
```

It uses PID control to maintain balanced speed between the two motors:

```c
error = pulses2 - pulses;

u = 20000 + kp*error; 

MAP_PWMPulseWidthSet(PWM_OUT_6, u);
MAP_PWMPulseWidthSet(PWM_OUT_7, y); 
```

Overall, the Python code provides high-level control while the C code handles lower-level motor actuation and feedback control. The UART serial communication links the two parts together.

## Important Files

"robot.py"
"uart_com_V1.c"
