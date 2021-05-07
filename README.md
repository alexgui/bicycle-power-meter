# Opposing Force Bicycle Power Meter
This project was completed as the capstone project for my [EE542 - Advanced Embedded Systems Design](https://class.ece.uw.edu/542/peckol/) course at the University of Washington during the Summer 2016 quarter. Contributors included Herman Chen and Muktar Rashid. The purpose of the project was to rapidly develop an embedded device and demonstrate end-to-end management of the development process from concept to working prototype.

As an avid cyclist, I turned to my main hobby for inspiration. I had always been interested in using power measurements to facilitate more effective training, so I figured this project was an excellent opportunity to see how far I could get in building my own opposing force power meter (OFBPM) from the ground up. Instead of taking a direct force measurement with strain gauges as most bicycle power meters do, an opposing force bicycle power meter operates on the fundamental principle of Newton's Third Law: for every action, there is an equal and opposite reaction. The amount of force required to move a bicycle and it's rider is equal to the sum of all opposing forces, such as aerodynamic drag, gravity, and friction.

An example of a commercially available opposing force bicycle power meter is the [Velocomp PowerPod](https://velocomp.com/powerpod-v4/).

A more detailed discussion of direct force versus opposing force power meters can be found [here](https://powermetercity.com/2016/04/04/direct-vs-opposing-force-power-meter/).

# Project Architecture
While Bluetooth Low Energy (BLE) is widely supported now, at the time of development the primary means of wireless communication for bicycle performance tracking was the ANT+ communication protocol. I had an existing Garmin Edge 510 GPS unit and Garmin GSC 10 wireless speed/cadence sensor, so the decision was made to leverage the existing hardware and build the OFBPM such that it could communicate directly with the Edge head unit.

The final project consisted of the following hardware:
- Garmin Edge 510 GPS Unit
- Nordic Semiconductor nRF52 Development Board
- Arduino Mega 2560
- Sensors
  - Garmin GSC 10 Speed/Cadence Sensor
  - Adafruit BMP280 (Temperature/Pressure Sensor)
  - Adafruit LIS3DH (Acceleromter)
  - Adafruit L3GD20H (Gyroscope)
  - Modern Devices Wind Sensor Rev. C (Anemometer)
- Breadboard
- Jumper Wires
- USB Battery Pack
- COTS Plastic Enclosure
- Mounting Hardware

The Arduino collects all of the raw sensor measurements, processes the data, and then passes the data to the nRF52. The nRF52 directly collects the speed/cadence data wirelessly via ANT+. Once all of the data is collected, the nRF52 performs the power calculation. Using the onboard ANT+ module, the nRF52 transmits the data to the head unit, where it can be displayed or combined with GPS data.

<img width="321" alt="Screen Shot 2021-05-06 at 3 46 42 PM" src="https://user-images.githubusercontent.com/10524839/117374864-cd671b00-ae82-11eb-897d-32ad18f80065.png">

<img width="570" alt="Screen Shot 2021-05-06 at 3 47 18 PM" src="https://user-images.githubusercontent.com/10524839/117374846-c50ee000-ae82-11eb-944a-64c3e9eb5558.png">

# Project Outcome
The project could successfully communicate with the Garmin Edge 510 GPS unit over ANT+ and demonstrated proportional effects to inputs such as wind, pitch, acceleration, etc. Data was compared to power readings from a PowerTap hub-based direct force bicycle power meter.

A full project report is available - just ask. :)

<img width="454" alt="Screen Shot 2021-05-06 at 3 47 33 PM" src="https://user-images.githubusercontent.com/10524839/117374824-bd4f3b80-ae82-11eb-882f-66e3755abe7f.png">

# Source Code
## Final Source Code
The following is a description of all of the source code used in the final device.

- main.c - This is the main file used for programming the nRF52. This file is responsible for reading the SAADC analog-to-digital converter on the nRF52 to receive power values from the Arduino, receiving bicycle speed/cadence inputs from the Garmin GSC 10, post processing the measurements, and transmitting the power value to the ANT+ bicycle power display device. This file uses ANT+ libraries available in the nRF5 Software Development Kit provided by Nordic Semiconductor.
- bikePower.ino - Arduino sketch that collects the data from the LIS3DH, L3GD20H, BMP280, and Wind Sensor Rev. C
- Sensor libraries
  - Adafruit_LIS3DH
  - Adafruit_L3GD20
  - Adafruit_BMP280
  - Adafruit_Sensor

## Partially Implemented Source Code
The original plan was to implement custom sensor libraries so the sensors could be directly read via the nRF52 (thus removing the Arduino from the hardware set), but difficulties in writing default states to the sensors and time constraints led us to abandon the custom libraries and leverage the pre-existing Arduino libraries.

- BMP280_main.c - Partially implemented custom nRF52 library for BMP280 temperature/pressure sensor
- lis3dh.c/h - Partially implemented custom nRF52 library for LIS3DH accelerometer
- l3gd20h.c/h - Partially implemented custom nRF52 library for L3GD20H gyroscope
- main_sensor.c - Main file used for reading from LIS3DH and L3GD20H sensors on the nRF52. It uses the lis3dh.c and l3gd20h.c library files listed above.
