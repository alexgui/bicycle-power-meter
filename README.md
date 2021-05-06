# Opposing Force Bicycle Power Meter
This project was completed as the capstone project for my [EE542 - Advanced Embedded Systems Design](https://class.ece.uw.edu/542/peckol/) course at the University of Washington. Contributors included Herman Chen and Muktar Rashid. The purpose of the project was to rapidly develop an embedded device.

As an avid cyclist, I turned to my main hobby for inspiration. I had always been interested in using power to facilitate more effective training, so I figured this project was an excellent opportunity to see how far I could get in building my own opposing force power meter (OFBPM) from the ground up. Instead of taking a direct force measurement with strain gauges as most bicycle power meters do, an opposing force bicycle power meter operates on the fundamental principle of Newton's Third Law: for every action, there is an equal and opposite reaction. The amount of force required to move a bicycle and it's rider is equal to the sum of all opposing forces, such as aerodynamic drag, gravity, and friction.

An example of a commercially available opposing force bicycle power meter is the [Velocomp PowerPod](https://velocomp.com/powerpod-v4/).

A more detailed discussion of direct force versus opposing force power meters can be found [here](https://powermetercity.com/2016/04/04/direct-vs-opposing-force-power-meter/).

# Project Architecture
While Bluetooth Low Energy (BLE) is widely supported now, at the time of development the primary means of wireless communication for bicycle performance tracking was the ANT+ communication protocol. I had an existing Garmin Edge 510 GPS unit and Garmin GSC 10 wireless speed/cadence sensor, so the decision was made to leverage the existing hardware and build the OFBPM such that it could communicate directly with the Edge head unit. The Edge would then combine all of the power data with the GPS stream and output it in the final .gpx file.

The final project consisted of the following hardware:
- Garmin Edge 510 GPS Unit
- Garmin GSC 10 Speed/Cadence Sensor
- Nordic Semiconductor
- Arduino
- Sensors
- Breadboard
- Jumper Wires
- COTS Plastic Enclosure
- Mounting Hardware
