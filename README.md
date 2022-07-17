# 3D-Scanning-System
Spatial Mapping Embedded System using Time of Flight sensor



An all-in-one spatial mapping embedded system that uses a time-of-flight sensor. It can scan an area with 360-degree planes along an orthogonal axis and display a 3D graphical representation on a PC running a Python program. It also stores XYZ coordinates in a separate file that may be used for further analysis.

This system encompasses a microcontroller unit (MCU) based on the Cortex M4, a stepper motor, and a ToF sensor. The system also needs a PC running prerequisite software. The MCU is responsible for processing data, as well as communicating with the ToF sensor and PC via I2C and UART, respectively. It also provides power to the various components mentioned above. The ToF sensor is responsible for data acquisition. It is mounted on the stepper motor, allowing it to capture data from a 360-degree plane. Finally, the PC is used to receive distance measurements, and store/display them for the user, as well as provide power to the MCU.

The VL53L1X measures distance using LIDAR. It emits a pulse of light with a wavelength of 940nm. The light will then hit the nearest object and return to the sensor. The component measures how long it takes for the emitted pulse of light to reach the object and return to the sensor. Using the time and speed of this pulse, distance can be obtained. It then uses I2C to transmit the distance data in millimeters to the MCU.
The included Python program waits for the user’s input, then allows the MCU and ToF sensor to boot. Once this process is complete, an onboard push button is used to start/stop data acquisition. After all planes have been scanned along the orthogonal axis, the included Python script will calculate the correct XYZ coordinates and save them to a file. This file is then used to generate a 3D visualization of the scan.

See 2DX_ProjectReport_josepc11.pdf for more detailed information.
 

![alt text](https://github.com/christy-josephanton/3D-Scanning-System/blob/main/Hallway_Sample.png?raw=true)
