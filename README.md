# Integrated Server Room Monitoring System

## Overview

This system combines environmental sensor monitoring with AI-powered visual monitoring to provide comprehensive security and safety monitoring for server rooms.

## Features

### Environmental Monitoring (Rack Level)
- **Temperature & Humidity** (AM2320 sensor)
- **Smoke/Gas Detection** (MQ2 sensor)
- **Current Monitoring** (ACS712 5A sensor)

### Visual Monitoring (Room Level)
- **Object Detection** (YOLO AI model)
- **Person Tracking** with unique IDs
- **Entry/Exit logging** with timestamps

### Event Correlation
- **Person Entry + Current Spike** → Someone powered on/off equipment
- **Person Entry + Temperature Spike** → Rack doors opened or equipment stress
- **Smoke Spike + Visual Confirmation** → Actual fire vs false alarm
- **Sensor Spike + No Person** → Equipment malfunction or unauthorized remote access

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Server Room                          │
│                                                         │
│  ┌──────────────┐         ┌─────────────────────┐     │
│  │   Camera     │         │   Server Rack       │     │
│  │   (Room)     │         │                     │     │
│  │              │         │  ┌──────────────┐   │     │
│  │  YOLO AI     │         │  │  AM2320      │   │     │
│  │  Detection   │         │  │  (Temp/Hum)  │   │     │
│  └──────────────┘         │  └──────────────┘   │     │
│                           │  ┌──────────────┐   │     │
│                           │  │  MQ2         │   │     │
│                           │  │  (Gas/Smoke) │   │     │
│                           │  └──────────────┘   │     │
│                           │  ┌──────────────┐   │     │
│                           │  │  ACS712      │   │     │
│                           │  │  (Current)   │   │     │
│                           │  └──────────────┘   │     │
│                           └─────────────────────┘     │
│                                                         │
│                     Raspberry Pi 4                      │
│              (Processing & Correlation)                 │
└─────────────────────────────────────────────────────────┘
```

## Hardware Requirements

1. **Raspberry Pi 4** (2GB+ RAM recommended)
2. **Raspberry Pi Camera Module** (or USB webcam)
3. **AM2320** Temperature & Humidity Sensor (I2C)
4. **MQ2** Gas/Smoke Sensor (Analog)
5. **ACS712 5A** Current Sensor (Analog)
6. **MCP3008** ADC (for analog sensors)
7. Jumper wires and breadboard

## Wiring Diagram

### I2C (AM2320)
```
AM2320          Raspberry Pi
VCC      →      3.3V (Pin 1)
GND      →      GND (Pin 6)
SDA      →      GPIO 2 (Pin 3)
SCL      →      GPIO 3 (Pin 5)
```

### SPI (MCP3008)
```
MCP3008         Raspberry Pi
VDD      →      3.3V
VREF     →      3.3V
AGND     →      GND
DGND     →      GND
CLK      →      GPIO 11 (SCLK)
DOUT     →      GPIO 9 (MISO)
DIN      →      GPIO 10 (MOSI)
CS       →      GPIO 8 (CE0)
```

### Analog Sensors to MCP3008
```
MQ2             MCP3008
A0       →      CH0

ACS712          MCP3008
OUT      →      CH1
```

## Installation

### Step 1: System Update
```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### Step 2: Enable Camera and I2C
```bash
sudo raspi-config
```
- Interface Options → Camera → Enable
- Interface Options → I2C → Enable
- Interface Options → SPI → Enable
- Reboot: `sudo reboot`

### Step 3: Install Python Dependencies
```bash
# Install system packages
sudo apt-get install -y python3-pip python3-opencv i2c-tools

# Install Python libraries
pip3 install opencv-python
pip3 install ultralytics
pip3 install spidev
pip3 install smbus2
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### Step 4: Verify Hardware Connections

**Test I2C:**
```bash
sudo i2cdetect -y 1
# Should show 0x5C for AM2320
```

**Test SPI:**
```bash
ls /dev/spidev*
# Should show /dev/spidev0.0
```

**Test Camera:**
```bash
libcamera-hello
# Should show camera preview
```

## Usage

### Run the Integrated System
```bash
cd ~/
python3 integrated_monitor.py
```

### First Run
- The YOLO model (6MB) will download automatically
- ACS712 will calibrate (remove any load first)
- Camera window will open showing live feed

### Output

**Console:**
```
[ENTRY] person detected (ID: 1)
Time: 2025-12-29 14:30:15

⚠️  CORRELATIONS DETECTED:
  • Current spike detected: 2.45A
  • Temperature increase: 42.3°C

--- Periodic Report (14:30:20) ---
Temp: 42.3°C | Humidity: 65.0% | Gas: 350 | Current: 2.45A
Active Objects: 1
```

**Files Created:**
```
monitoring_logs/
├── events/
│   ├── object_detected_20251229_143015_123456.json
│   └── object_exit_20251229_143045_789012.json
├── snapshots/
│   └── object_detected_20251229_143015_123456.jpg
└── sensor_logs/
    └── report_20251229_143020.json
```

### Example Event File
```json
{
  "event_id": "object_detected_20251229_143015_123456",
  "event_type": "object_detected",
  "timestamp": "2025-12-29T14:30:15.123456",
  "details": {
    "object_type": "person",
    "object_id": 1,
    "confidence": 0.87,
    "action": "entry",
    "sensor_data": {
      "temperature": 42.3,
      "humidity": 65.0,
      "gas_raw": 350,
      "current": 2.45
    },
    "correlations": [
      {
        "type": "current_spike",
        "value": 2.45,
        "message": "Current spike detected: 2.45A"
      },
      {
        "type": "temperature_spike",
        "value": 42.3,
        "message": "Temperature increase: 42.3°C"
      }
    ]
  },
  "snapshot": "monitoring_logs/snapshots/object_detected_20251229_143015_123456.jpg"
}
```

## Key Correlations Detected

### 1. **Authorized Maintenance**
- Person enters → Current spike → Normal
- Indicates: Equipment being serviced

### 2. **Unauthorized Access**
- Person enters → No sensor activity
- Indicates: Possible unauthorized entry (not touching equipment)

### 3. **Equipment Issue**
- Current spike + Temperature spike → No person
- Indicates: Equipment malfunction or overload

### 4. **Fire/Smoke Alert**
- Gas spike + Visual confirmation
- Indicates: Actual fire emergency vs sensor malfunction

### 5. **Door Left Open**
- Person exits → Temperature rises slowly
- Indicates: Rack door or room door left open

## Configuration

Edit thresholds in `integrated_monitor.py`:

```python
# Sensor Thresholds
GAS_THRESHOLD = 700              # Smoke alert level
TEMP_THRESHOLD = 40.0            # Temperature °C
HUM_THRESHOLD = 80.0             # Humidity %
CURRENT_SPIKE_THRESHOLD = 1.5    # Amps

# Correlation Settings
CORRELATION_WINDOW = 10          # seconds
PERSON_TIMEOUT = 5               # seconds before session ends
```

## Running as Background Service

Create systemd service:

```bash
sudo nano /etc/systemd/system/server-monitor.service
```

Add:
```ini
[Unit]
Description=Server Room Monitoring System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStart=/usr/bin/python3 /home/pi/integrated_monitor.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable server-monitor
sudo systemctl start server-monitor
sudo systemctl status server-monitor
```

## Performance Optimization

For better performance on Raspberry Pi 4:

1. **Lower camera resolution:**
```python
self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 416)
self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 416)
```

2. **Reduce processing frequency:**
```python
CAMERA_PROCESS_INTERVAL = 0.2  # Process every 200ms
```

3. **Use smaller YOLO model:**
```python
self.model = YOLO('yolov8n.pt')  # Already using nano
```

## Troubleshooting

### Camera not working
```bash
# Test camera
libcamera-hello

# Check if detected
vcgencmd get_camera
```

### I2C not working
```bash
# Check device
sudo i2cdetect -y 1

# Enable I2C
sudo raspi-config
```

### High CPU usage
- Comment out `cv2.imshow()` for headless operation
- Increase `CAMERA_PROCESS_INTERVAL`
- Use lower camera resolution

### Sensor readings stuck
- Check wiring connections
- Verify power supply (5V 3A recommended)
- Test individual sensors separately

## Data Analysis

View events:
```bash
# List all events
ls -lh monitoring_logs/events/

# View specific event
cat monitoring_logs/events/object_detected_*.json | jq .

# Count person entries today
ls monitoring_logs/events/object_detected_*.json | wc -l
```

## Security Recommendations

1. **Alert on unauthorized access:**
   - Person detected outside work hours
   - Multiple current spikes without person

2. **Monitor trends:**
   - Gradually increasing temperature
   - Frequent gas sensor spikes

3. **Backup logs regularly:**
```bash
# Daily backup
tar -czf logs_backup_$(date +%Y%m%d).tar.gz monitoring_logs/
```

## Future Enhancements

- [ ] Send email/SMS alerts on critical events
- [ ] Web dashboard for real-time monitoring
- [ ] Database storage (SQLite/PostgreSQL)
- [ ] Face recognition for access control
- [ ] Predictive maintenance based on trends
- [ ] Mobile app integration

## License

MIT License

## Support

For issues or questions, check the hardware connections and verify all sensors are working individually before running the integrated system.
