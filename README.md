# IoT-environment-monitor
# NodeMCU IoT Environment Monitor with MQTT and Auto Reset

An IoT-based monitoring system using the NodeMCU to track temperature,humidity, and electrical current, and to automatically reset daily using the internal RTC. The ESP32 sends sensor data via MQTT to a cloud broker, making it ideal for remote monitoring in smart homes, energy management, or environmental tracking applications.



# Features

- Temperature & Humidity Monitoring (DHT22)
- Current Sensing using ACS712 (30A)
- MQTT Communication*for real-time data transmission
- Daily Auto Reset at a configurable time using RTC
- Mobile App Integration via MQTT Dashboard or custom Flutter app
- Real-time Monitoring** and Alert Support
- Secure Wi-Fi Configuration 
- Low Power-Friendly Setup (sleep support optional)



# Hardware Components

 Component          Description                      

 ESP32 Dev Board   Main microcontroller             
 DHT22             Temperature and humidity sensor  
 ACS712 (30A)      Current sensor                   
 Power Supply      5V regulated source              
 Wi-Fi Access      Required for MQTT connectivity   


# MQTT Configuration

- Broker: HiveMQ public broker (or custom)
- Port: '1883' (or '8883' for TLS)
- Topics Published:
  - `mytopic/temp`
  - `mytopic/humidity`
  - `mytopic/current`
  - `mytopic/status`

You can subscribe to these topics using MQTT Dash, Flutter App, or any MQTT-compatible client.



#  Auto Reset (via RTC)

The ESP32 uses its internal Real-Time Clock (RTC) to automatically reset once every 1 minute to prevent sensor hangs, Wi-Fi issues, or memory leaks. This reset helps ensure long-term reliability of the system in unattended deployments.

You can customize the reset time using the `esp_restart()` function and RTC time checks in code.



#  Mobile App Integration and Web Dashboard

- Use MQTT Dash or a custom Flutter app to:
  - Display live data
  - Set thresholds (optional)
  - Get alerts on abnormal values
  - Storing data


# Version
-there is 2 versions of the code project :
1-Code wrote with emebedded C using ESP-IDF
2-Code wrote with Arduino IDE

