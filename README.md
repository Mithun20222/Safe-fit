SafeFit – Smart Health & Women Safety Bracelet
SafeFit is a smart wearable bracelet that I am building to combine health monitoring and women’s safety into a single compact device.
The idea behind SafeFit is to create a wearable that can continuously track vital health parameters and also act as an emergency safety device, even when there is no internet or smartphone available.
This project focuses on real-world problems such as delayed medical response, lack of affordable health wearables, and the need for a reliable SOS system for women and elderly people.

Why SafeFit?
Most existing fitness bands focus only on steps and heart rate, while safety devices focus only on alerts. SafeFit brings both health and safety together in one device.
The key goals of this project are:
Continuous health tracking
Instant emergency alerts
Location sharing during emergencies
Independent operation without depending on Wi-Fi or mobile apps

Features
Health & Fitness
Heart rate monitoring using MAX30102
SpO₂ (oxygen level) estimation
Step counting using motion data
Activity and movement analysis

Fall detection using accelerometer and gyroscope
Safety & Emergency
Automatic fall detection
SOS alert system using GSM (SIM800L)
GPS-based live location sharing
Works without an internet or smartphone connection

System Overview
SafeFit is built using a combination of sensors, communication modules, and a microcontroller.

Hardware Used
NodeMCU (ESP8266) – main controller
MAX30102 – heart rate and SpO₂ sensor
MPU6050 – accelerometer and gyroscope
GPS module (NEO-6M)
SIM800L GSM module
18650 Li-ion battery
TP4056 charging module
Boost and voltage regulators

Software & Logic
Arduino framework
TinyGPS++ for GPS data
MAX3010x library for heart rate and SpO₂
Digital signal filtering for accurate readings
Logic for step counting, fall detection, and SOS triggering

Power Management
The system is powered using a 18650 Li-ion battery.
SIM800L is powered directly from the battery to handle high current peaks.
NodeMCU and GPS are powered through a boosted 5V line.
Sensors are powered using a regulated 3.3V supply.
A TP4056 module is used for safe charging and battery protection.
All components share a common ground to ensure stable communication.

Pin Connections (Summary)
I²C Sensors
MAX30102 → SDA: D2, SCL: D1
MPU6050 → SDA: D2, SCL: D1
Serial Communication
GPS → TX: D6, RX: D5
SIM800L → TX: D8, RX: D7
How the System Works
Sensors continuously collect health and motion data.
Step count and movement patterns are calculated using MPU6050.
Heart rate and SpO₂ are processed using digital filters.
GPS continuously updates the location.
If a fall or emergency is detected:
The SIM800L sends an SOS SMS with live location.
Health and safety data can be viewed through the serial monitor and extended to apps in the future.

How to Run the Project
Connect all hardware components as per the wiring.
Insert an active SIM card into the SIM800L module.
Power the system using a charged 18650 battery.
Upload the code using the Arduino IDE.
Open the Serial Monitor at 115200 baud.
Place a finger on the MAX30102 sensor to see heart rate and SpO₂ values.
Test fall detection and SOS functionality.

Important Notes
SIM800L requires high current during transmission, so proper power supply and capacitors are important.
GPS requires open sky for faster and accurate location lock.
Images used in documentation are development or conceptual models.
This project is currently under development and will be optimized further.

Future Improvements
Mobile application integration
Cloud-based health data storage
AI-based health anomaly detection
Voice-activated SOS
Geofencing for children and elderly
Compact PCB and wearable enclosure design

About Me
Mithun Gullapalli
B.Tech CSE (AI & ML)
VIT-AP University

License
This project is open for learning, academic use, and research purposes.

Demo
https://drive.google.com/file/d/1qOGTfLUYuJ50gstK8vR3eZCe-cBDzhBj/view?usp=sharing
