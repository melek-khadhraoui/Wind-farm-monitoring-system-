# Wind Farm Monitoring System

## Design and Implementation of an Embedded Monitoring System for a Wind Farm

### Objective
To develop a system for real-time monitoring of wind turbine performance, weather conditions, and the status of the electrical grid.

### Key Steps

#### Real-Time Data Collection:
- **Wind Turbine Performance**:
  - Monitoring blade speed.
  - Energy production (voltage, current).
  - Component monitoring (vibration, temperature).
  
- **Weather Conditions**:
  - Measurement of wind speed, temperature, and pressure to optimize wind turbine performance.
  
- **Electrical Grid Status**:
  - Monitoring electrical parameters (voltage, current) to ensure integration into the grid.

#### Embedded Architecture:
- **STM32**:
  - Management of sensors and local data processing.
  
- **ESP32**:
  - Data transmission via Wi-Fi and management of Bluetooth connectivity for diagnostics or local updates.
  
- **Node-RED**:
  - Visualization of data and creation of interactive dashboards for wind turbine supervision.

#### Network and Communication:
- **Wi-Fi (via ESP32)**:
  - Transmission of collected data to the central station.
  
- **Bluetooth (via ESP32)**:
  - Local communication for wind turbine maintenance.
  
- **MQTT/WebSocket**:
  - Protocols for rapid and efficient communication with the central control station.

#### Analysis and Visualization:
- **Embedded Preprocessing**:
  - On-site anomaly detection.
  
- **Dashboards (Node-RED)**:
  - Real-time display of collected data for decision-making.

### Skills and Technologies:
- **Microcontrollers**: STM32
- **Wireless Communication**: ESP32, Wi-Fi, Bluetooth
- **Data Visualization**: Node-RED
- **Communication Protocols**: MQTT, WebSocket

### Presentations
I have created two presentations for this project in French to explain the objectives, design, and functionality of the monitoring system.
