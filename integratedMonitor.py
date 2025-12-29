#!/usr/bin/env python3
"""
Integrated Server Room Monitoring System
Combines AI object detection/tracking with environmental sensors

Features:
- Visual monitoring (camera + YOLO)
- Environmental monitoring (temp, humidity, smoke, current)
- Event correlation (person entry vs sensor spikes)
- Unified logging and alerting
"""

import cv2
from ultralytics import YOLO
import datetime
import json
import os
import threading
import time
import spidev
import smbus2 as smbus
from collections import deque
from pathlib import Path


# ============================================================================
# CONFIGURATION
# ============================================================================

# Sensor Thresholds
GAS_THRESHOLD = 700
TEMP_THRESHOLD = 40.0
HUM_THRESHOLD = 80.0
CURRENT_SPIKE_THRESHOLD = 1.5  # Amps - significant change

# I2C Configuration
I2C_ADDR = 0x5C
I2C_BUS = 1

# SPI Configuration
SPI_BUS = 0
SPI_DEVICE = 0

# ADC Configuration
VREF = 3.3
ADC_MAX = 1023.0

# ACS712 Configuration
ACS_CHANNEL = 1
ACS_SENSITIVITY = 0.185
ACS_CAL_SAMPLES = 100
ACS_READ_SAMPLES = 30

# MQ2 Configuration
MQ2_CHANNEL = 0

# Sensor Reading Intervals
SENSOR_READ_INTERVAL = 1.0
CAMERA_PROCESS_INTERVAL = 0.1

# Correlation Settings
CORRELATION_WINDOW = 10  # seconds - window to correlate events
PERSON_TIMEOUT = 5  # seconds before person session ends

# Output directories
OUTPUT_DIR = Path("monitoring_logs")
EVENTS_DIR = OUTPUT_DIR / "events"
SNAPSHOTS_DIR = OUTPUT_DIR / "snapshots"
SENSOR_LOGS_DIR = OUTPUT_DIR / "sensor_logs"

# Create directories
for dir_path in [OUTPUT_DIR, EVENTS_DIR, SNAPSHOTS_DIR, SENSOR_LOGS_DIR]:
    dir_path.mkdir(parents=True, exist_ok=True)


# ============================================================================
# SHARED STATE
# ============================================================================

sensor_data = {
    "temperature": None,
    "humidity": None,
    "mq2_raw": None,
    "mq2_voltage": None,
    "acs_current": None,
    "timestamp": None
}

camera_data = {
    "active_objects": {},  # track_id -> object info
    "recent_entries": deque(maxlen=100),  # Recent person entries
    "recent_exits": deque(maxlen=100)
}

# Historical data for spike detection
sensor_history = {
    "current": deque(maxlen=60),  # Last 60 readings
    "temperature": deque(maxlen=60),
    "gas": deque(maxlen=60)
}

data_lock = threading.Lock()
stop_event = threading.Event()


# ============================================================================
# SPI / MCP3008 SETUP
# ============================================================================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 1350000


def read_mcp3008_channel(channel):
    """Read analog value from MCP3008 ADC"""
    resp = spi.xfer2([1, (8 + channel) << 4, 0])
    value = ((resp[1] & 3) << 8) | resp[2]
    return value


# ============================================================================
# SENSOR READERS
# ============================================================================

class AM2320Reader(threading.Thread):
    """Thread for reading AM2320 temperature and humidity sensor"""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.bus = smbus.SMBus(I2C_BUS)

    def read_am2320(self):
        try:
            self.bus.write_i2c_block_data(I2C_ADDR, 0x00, [])
        except:
            pass
        time.sleep(0.003)

        self.bus.write_i2c_block_data(I2C_ADDR, 0x03, [0x00, 0x04])
        time.sleep(0.002)

        data = self.bus.read_i2c_block_data(I2C_ADDR, 0, 6)

        humidity = ((data[2] << 8) | data[3]) / 10.0
        temp_raw = (data[4] << 8) | data[5]

        if temp_raw & 0x8000:
            temp_raw = -(temp_raw & 0x7FFF)

        temperature = temp_raw / 10.0
        return temperature, humidity

    def run(self):
        while not stop_event.is_set():
            try:
                temp, hum = self.read_am2320()
                timestamp = datetime.datetime.now()
                
                with data_lock:
                    sensor_data["temperature"] = temp
                    sensor_data["humidity"] = hum
                    sensor_data["timestamp"] = timestamp
                    
                    # Add to history
                    sensor_history["temperature"].append({
                        "value": temp,
                        "timestamp": timestamp
                    })
                    
            except Exception as e:
                print(f"[AM2320] Error: {e}")

            time.sleep(SENSOR_READ_INTERVAL)


class MQ2Reader(threading.Thread):
    """Thread for reading MQ2 gas/smoke sensor"""
    
    def run(self):
        while not stop_event.is_set():
            try:
                raw = read_mcp3008_channel(MQ2_CHANNEL)
                voltage = (raw / ADC_MAX) * VREF
                timestamp = datetime.datetime.now()

                with data_lock:
                    sensor_data["mq2_raw"] = raw
                    sensor_data["mq2_voltage"] = voltage
                    
                    # Add to history
                    sensor_history["gas"].append({
                        "value": raw,
                        "timestamp": timestamp
                    })

            except Exception as e:
                print(f"[MQ2] Error: {e}")

            time.sleep(SENSOR_READ_INTERVAL)


class ACS712Reader(threading.Thread):
    """Thread for reading ACS712 current sensor"""
    
    def __init__(self):
        super().__init__(daemon=True)
        self.zero_voltage = None

    def calibrate_zero(self):
        vals = []
        print("[ACS712] Calibrating zero current...")
        for _ in range(ACS_CAL_SAMPLES):
            raw = read_mcp3008_channel(ACS_CHANNEL)
            v = (raw / ADC_MAX) * VREF
            vals.append(v)
            time.sleep(0.01)

        self.zero_voltage = sum(vals) / len(vals)
        print(f"[ACS712] Zero point = {self.zero_voltage:.4f} V")

    def read_current(self):
        samples = []
        for _ in range(ACS_READ_SAMPLES):
            raw = read_mcp3008_channel(ACS_CHANNEL)
            v = (raw / ADC_MAX) * VREF
            samples.append(v)
            time.sleep(0.002)

        avg_v = sum(samples) / len(samples)
        shifted = avg_v - self.zero_voltage
        current = shifted / ACS_SENSITIVITY

        if current < 0:
            current = 0

        return current

    def run(self):
        try:
            self.calibrate_zero()
        except:
            self.zero_voltage = 2.5

        while not stop_event.is_set():
            try:
                current = self.read_current()
                timestamp = datetime.datetime.now()
                
                with data_lock:
                    sensor_data["acs_current"] = current
                    
                    # Add to history
                    sensor_history["current"].append({
                        "value": current,
                        "timestamp": timestamp
                    })

            except Exception as e:
                print(f"[ACS712] Error: {e}")

            time.sleep(0.2)


# ============================================================================
# CAMERA / AI DETECTION
# ============================================================================

class CameraMonitor(threading.Thread):
    """Thread for camera monitoring and object detection"""
    
    def __init__(self):
        super().__init__(daemon=True)
        print("[CAMERA] Loading YOLO model...")
        self.model = YOLO('yolov8n.pt')
        print("[CAMERA] Model loaded!")
        
        print("[CAMERA] Initializing camera...")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            raise Exception("Could not open camera")
        
        print("[CAMERA] Camera initialized!")
        
        self.frame_count = 0

    def detect_spike(self, history_key, current_value, threshold_multiplier=1.5):
        """Detect if current value is a spike compared to recent history"""
        with data_lock:
            history = list(sensor_history[history_key])
        
        if len(history) < 5:
            return False
        
        recent_values = [h["value"] for h in history[-10:]]
        avg = sum(recent_values) / len(recent_values)
        
        return current_value > avg * threshold_multiplier

    def save_event(self, event_type, details, frame=None):
        """Save event to log with optional snapshot"""
        timestamp = datetime.datetime.now()
        event_id = f"{event_type}_{timestamp.strftime('%Y%m%d_%H%M%S_%f')}"
        
        event_data = {
            "event_id": event_id,
            "event_type": event_type,
            "timestamp": timestamp.isoformat(),
            "details": details
        }
        
        # Save snapshot if frame provided
        if frame is not None:
            snapshot_path = SNAPSHOTS_DIR / f"{event_id}.jpg"
            cv2.imwrite(str(snapshot_path), frame)
            event_data["snapshot"] = str(snapshot_path)
        
        # Save event JSON
        event_path = EVENTS_DIR / f"{event_id}.json"
        with open(event_path, 'w') as f:
            json.dump(event_data, f, indent=2)
        
        return event_data

    def correlate_events(self, object_type, action):
        """Correlate visual events with sensor readings"""
        timestamp = datetime.datetime.now()
        correlations = []
        
        with data_lock:
            current = sensor_data["acs_current"]
            temp = sensor_data["temperature"]
            gas = sensor_data["mq2_raw"]
        
        # Check for current spike
        if current and self.detect_spike("current", current, 1.3):
            correlations.append({
                "type": "current_spike",
                "value": current,
                "message": f"Current spike detected: {current:.2f}A"
            })
        
        # Check for temperature spike
        if temp and self.detect_spike("temperature", temp, 1.1):
            correlations.append({
                "type": "temperature_spike",
                "value": temp,
                "message": f"Temperature increase: {temp:.1f}°C"
            })
        
        # Check for gas spike
        if gas and self.detect_spike("gas", gas, 1.2):
            correlations.append({
                "type": "gas_spike",
                "value": gas,
                "message": f"Gas level spike: {gas}"
            })
        
        return correlations

    def run(self):
        while not stop_event.is_set():
            ret, frame = self.cap.read()
            
            if not ret:
                print("[CAMERA] Failed to capture frame")
                time.sleep(0.1)
                continue
            
            self.frame_count += 1
            timestamp = datetime.datetime.now()
            
            # Run detection with tracking
            results = self.model.track(frame, persist=True, verbose=False)
            
            current_ids = []
            
            # Process detections
            if results[0].boxes is not None and len(results[0].boxes) > 0:
                for detection in results[0].boxes:
                    track_id = int(detection.id[0]) if detection.id is not None else None
                    class_id = int(detection.cls[0])
                    class_name = self.model.names[class_id]
                    confidence = float(detection.conf[0])
                    
                    if confidence < 0.5 or not track_id:
                        continue
                    
                    current_ids.append(track_id)
                    
                    with data_lock:
                        # New object detected
                        if track_id not in camera_data["active_objects"]:
                            obj_info = {
                                "id": track_id,
                                "type": class_name,
                                "first_seen": timestamp,
                                "last_seen": timestamp,
                                "confidence": confidence
                            }
                            camera_data["active_objects"][track_id] = obj_info
                            
                            # Correlate with sensors
                            correlations = self.correlate_events(class_name, "entry")
                            
                            # Save event
                            event_details = {
                                "object_type": class_name,
                                "object_id": track_id,
                                "confidence": confidence,
                                "action": "entry",
                                "sensor_data": {
                                    "temperature": sensor_data["temperature"],
                                    "humidity": sensor_data["humidity"],
                                    "gas_raw": sensor_data["mq2_raw"],
                                    "current": sensor_data["acs_current"]
                                },
                                "correlations": correlations
                            }
                            
                            event = self.save_event("object_detected", event_details, frame)
                            
                            # Log to console
                            alert_msg = f"\n{'='*60}\n"
                            alert_msg += f"[ENTRY] {class_name} detected (ID: {track_id})\n"
                            alert_msg += f"Time: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}\n"
                            
                            if correlations:
                                alert_msg += "\n⚠️  CORRELATIONS DETECTED:\n"
                                for corr in correlations:
                                    alert_msg += f"  • {corr['message']}\n"
                            
                            alert_msg += f"{'='*60}\n"
                            print(alert_msg)
                            
                            if class_name == "person":
                                camera_data["recent_entries"].append({
                                    "id": track_id,
                                    "timestamp": timestamp
                                })
                        
                        # Update last seen
                        else:
                            camera_data["active_objects"][track_id]["last_seen"] = timestamp
            
            # Check for objects that left
            with data_lock:
                for track_id in list(camera_data["active_objects"].keys()):
                    if track_id not in current_ids:
                        obj = camera_data["active_objects"][track_id]
                        time_since_seen = (timestamp - obj["last_seen"]).total_seconds()
                        
                        if time_since_seen > PERSON_TIMEOUT:
                            # Object left
                            obj_info = camera_data["active_objects"].pop(track_id)
                            duration = (timestamp - obj_info["first_seen"]).total_seconds()
                            
                            # Save exit event
                            exit_details = {
                                "object_type": obj_info["type"],
                                "object_id": track_id,
                                "action": "exit",
                                "duration_seconds": duration,
                                "entry_time": obj_info["first_seen"].isoformat()
                            }
                            
                            self.save_event("object_exit", exit_details)
                            
                            print(f"[EXIT] {obj_info['type']} (ID: {track_id}) - Duration: {duration:.1f}s")
                            
                            if obj_info["type"] == "person":
                                camera_data["recent_exits"].append({
                                    "id": track_id,
                                    "timestamp": timestamp
                                })
            
            # Display frame (optional - comment out for headless)
            annotated_frame = results[0].plot()
            
            # Add sensor overlay
            y_offset = 30
            with data_lock:
                overlay_text = [
                    f"Temp: {sensor_data['temperature']:.1f}°C" if sensor_data['temperature'] else "Temp: --",
                    f"Humidity: {sensor_data['humidity']:.1f}%" if sensor_data['humidity'] else "Humidity: --",
                    f"Gas: {sensor_data['mq2_raw']}" if sensor_data['mq2_raw'] else "Gas: --",
                    f"Current: {sensor_data['acs_current']:.2f}A" if sensor_data['acs_current'] else "Current: --"
                ]
            
            for text in overlay_text:
                cv2.putText(annotated_frame, text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                y_offset += 25
            
            cv2.imshow('Server Room Monitor', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break
            
            time.sleep(CAMERA_PROCESS_INTERVAL)
        
        self.cap.release()
        cv2.destroyAllWindows()


# ============================================================================
# PERIODIC REPORTER
# ============================================================================

class PeriodicReporter(threading.Thread):
    """Thread for periodic sensor reports"""
    
    def run(self):
        report_interval = 10  # seconds
        
        while not stop_event.is_set():
            time.sleep(report_interval)
            
            timestamp = datetime.datetime.now()
            
            with data_lock:
                report = {
                    "timestamp": timestamp.isoformat(),
                    "sensors": {
                        "temperature": sensor_data["temperature"],
                        "humidity": sensor_data["humidity"],
                        "gas_raw": sensor_data["mq2_raw"],
                        "gas_voltage": sensor_data["mq2_voltage"],
                        "current": sensor_data["acs_current"]
                    },
                    "camera": {
                        "active_objects": len(camera_data["active_objects"]),
                        "objects": [
                            {
                                "id": obj["id"],
                                "type": obj["type"],
                                "duration": (timestamp - obj["first_seen"]).total_seconds()
                            }
                            for obj in camera_data["active_objects"].values()
                        ]
                    }
                }
            
            # Save periodic report
            report_file = SENSOR_LOGS_DIR / f"report_{timestamp.strftime('%Y%m%d_%H%M%S')}.json"
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            
            # Console output
            print(f"\n--- Periodic Report ({timestamp.strftime('%H:%M:%S')}) ---")
            print(f"Temp: {report['sensors']['temperature']:.1f}°C | "
                  f"Humidity: {report['sensors']['humidity']:.1f}% | "
                  f"Gas: {report['sensors']['gas_raw']} | "
                  f"Current: {report['sensors']['current']:.2f}A")
            print(f"Active Objects: {report['camera']['active_objects']}")


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("\n" + "="*60)
    print("SERVER ROOM INTEGRATED MONITORING SYSTEM")
    print("="*60 + "\n")
    
    # Start sensor threads
    print("Starting sensor threads...")
    am_reader = AM2320Reader()
    mq_reader = MQ2Reader()
    acs_reader = ACS712Reader()
    
    am_reader.start()
    mq_reader.start()
    acs_reader.start()
    
    # Wait for initial sensor readings
    time.sleep(2)
    
    # Start camera monitor
    print("Starting camera monitor...")
    camera_monitor = CameraMonitor()
    camera_monitor.start()
    
    # Start periodic reporter
    print("Starting periodic reporter...")
    reporter = PeriodicReporter()
    reporter.start()
    
    print("\n✓ All systems running!")
    print("Press Ctrl+C to stop\n")
    
    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        stop_event.set()
        time.sleep(1)
        spi.close()
        print("System stopped.")


if __name__ == "__main__":
    main()
