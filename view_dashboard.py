#!/usr/bin/env python3
"""
Simple Dashboard for Server Room Monitoring System
View events, correlations, and statistics
"""

import json
import os
from pathlib import Path
from datetime import datetime, timedelta
from collections import Counter
import argparse


def load_events(events_dir):
    """Load all event files"""
    events = []
    events_path = Path(events_dir)
    
    if not events_path.exists():
        return events
    
    for event_file in events_path.glob("*.json"):
        try:
            with open(event_file, 'r') as f:
                event = json.load(f)
                events.append(event)
        except:
            continue
    
    # Sort by timestamp
    events.sort(key=lambda x: x['timestamp'])
    return events


def print_event_summary(events):
    """Print summary of all events"""
    if not events:
        print("No events found.")
        return
    
    print("\n" + "="*80)
    print("EVENT SUMMARY")
    print("="*80)
    
    # Count event types
    event_types = Counter([e['event_type'] for e in events])
    
    print(f"\nTotal Events: {len(events)}")
    for event_type, count in event_types.items():
        print(f"  {event_type}: {count}")
    
    # Count object types
    object_types = []
    for e in events:
        if 'details' in e and 'object_type' in e['details']:
            object_types.append(e['details']['object_type'])
    
    if object_types:
        print(f"\nDetected Objects:")
        for obj_type, count in Counter(object_types).items():
            print(f"  {obj_type}: {count}")


def print_correlations(events):
    """Print all events with correlations"""
    corr_events = [e for e in events 
                   if 'details' in e 
                   and 'correlations' in e['details'] 
                   and e['details']['correlations']]
    
    if not corr_events:
        print("\nNo correlations detected.")
        return
    
    print("\n" + "="*80)
    print("CORRELATED EVENTS")
    print("="*80)
    
    for event in corr_events:
        details = event['details']
        timestamp = datetime.fromisoformat(event['timestamp'])
        
        print(f"\n[{timestamp.strftime('%Y-%m-%d %H:%M:%S')}]")
        print(f"Object: {details['object_type']} (ID: {details['object_id']})")
        print(f"Action: {details['action']}")
        
        print("\nSensor Readings:")
        sensor_data = details['sensor_data']
        print(f"  Temperature: {sensor_data['temperature']:.1f}°C")
        print(f"  Humidity: {sensor_data['humidity']:.1f}%")
        print(f"  Gas: {sensor_data['gas_raw']}")
        print(f"  Current: {sensor_data['current']:.2f}A")
        
        print("\n⚠️  Correlations:")
        for corr in details['correlations']:
            print(f"  • {corr['message']}")
        
        if 'snapshot' in event:
            print(f"\nSnapshot: {event['snapshot']}")
        
        print("-" * 80)


def print_recent_events(events, hours=1):
    """Print events from last N hours"""
    now = datetime.now()
    cutoff = now - timedelta(hours=hours)
    
    recent = [e for e in events 
              if datetime.fromisoformat(e['timestamp']) > cutoff]
    
    if not recent:
        print(f"\nNo events in the last {hours} hour(s).")
        return
    
    print("\n" + "="*80)
    print(f"RECENT EVENTS (Last {hours} hour(s))")
    print("="*80)
    
    for event in recent:
        timestamp = datetime.fromisoformat(event['timestamp'])
        print(f"\n[{timestamp.strftime('%H:%M:%S')}] {event['event_type']}")
        
        if 'details' in event:
            details = event['details']
            if 'object_type' in details:
                print(f"  Object: {details['object_type']} (ID: {details.get('object_id', 'N/A')})")
            if 'action' in details:
                print(f"  Action: {details['action']}")
            if 'duration_seconds' in details:
                print(f"  Duration: {details['duration_seconds']:.1f}s")


def print_person_sessions(events):
    """Print summary of person entry/exit sessions"""
    entries = [e for e in events 
               if e['event_type'] == 'object_detected'
               and e['details']['object_type'] == 'person']
    
    exits = [e for e in events 
             if e['event_type'] == 'object_exit'
             and e['details']['object_type'] == 'person']
    
    if not entries:
        print("\nNo person detections found.")
        return
    
    print("\n" + "="*80)
    print("PERSON SESSIONS")
    print("="*80)
    
    print(f"\nTotal Entries: {len(entries)}")
    print(f"Total Exits: {len(exits)}")
    
    # Match entries with exits
    sessions = []
    for entry in entries:
        entry_id = entry['details']['object_id']
        entry_time = datetime.fromisoformat(entry['timestamp'])
        
        # Find corresponding exit
        exit_event = None
        for exit in exits:
            if exit['details']['object_id'] == entry_id:
                exit_event = exit
                break
        
        if exit_event:
            exit_time = datetime.fromisoformat(exit_event['timestamp'])
            duration = (exit_time - entry_time).total_seconds()
        else:
            duration = None
        
        sessions.append({
            'id': entry_id,
            'entry_time': entry_time,
            'duration': duration,
            'had_correlations': len(entry['details'].get('correlations', [])) > 0
        })
    
    # Print sessions
    print(f"\nDetailed Sessions:")
    for session in sessions[-10:]:  # Last 10 sessions
        print(f"\n  ID: {session['id']}")
        print(f"  Entry: {session['entry_time'].strftime('%Y-%m-%d %H:%M:%S')}")
        if session['duration']:
            print(f"  Duration: {session['duration']:.1f}s")
        else:
            print(f"  Duration: Still active")
        if session['had_correlations']:
            print(f"  ⚠️  Had sensor correlations")


def print_sensor_stats(events):
    """Print sensor statistics from events"""
    sensor_readings = []
    
    for event in events:
        if 'details' in event and 'sensor_data' in event['details']:
            sensor_readings.append(event['details']['sensor_data'])
    
    if not sensor_readings:
        print("\nNo sensor data found.")
        return
    
    print("\n" + "="*80)
    print("SENSOR STATISTICS")
    print("="*80)
    
    # Calculate averages
    temps = [s['temperature'] for s in sensor_readings if s['temperature']]
    hums = [s['humidity'] for s in sensor_readings if s['humidity']]
    gas_vals = [s['gas_raw'] for s in sensor_readings if s['gas_raw']]
    currents = [s['current'] for s in sensor_readings if s['current']]
    
    if temps:
        print(f"\nTemperature:")
        print(f"  Average: {sum(temps)/len(temps):.1f}°C")
        print(f"  Min: {min(temps):.1f}°C")
        print(f"  Max: {max(temps):.1f}°C")
    
    if hums:
        print(f"\nHumidity:")
        print(f"  Average: {sum(hums)/len(hums):.1f}%")
        print(f"  Min: {min(hums):.1f}%")
        print(f"  Max: {max(hums):.1f}%")
    
    if gas_vals:
        print(f"\nGas Sensor:")
        print(f"  Average: {sum(gas_vals)/len(gas_vals):.0f}")
        print(f"  Min: {min(gas_vals)}")
        print(f"  Max: {max(gas_vals)}")
    
    if currents:
        print(f"\nCurrent:")
        print(f"  Average: {sum(currents)/len(currents):.2f}A")
        print(f"  Min: {min(currents):.2f}A")
        print(f"  Max: {max(currents):.2f}A")


def main():
    parser = argparse.ArgumentParser(description='Server Room Monitoring Dashboard')
    parser.add_argument('--events-dir', default='monitoring_logs/events',
                       help='Path to events directory')
    parser.add_argument('--summary', action='store_true',
                       help='Show event summary')
    parser.add_argument('--correlations', action='store_true',
                       help='Show correlated events')
    parser.add_argument('--recent', type=int, metavar='HOURS',
                       help='Show events from last N hours')
    parser.add_argument('--sessions', action='store_true',
                       help='Show person sessions')
    parser.add_argument('--stats', action='store_true',
                       help='Show sensor statistics')
    parser.add_argument('--all', action='store_true',
                       help='Show all reports')
    
    args = parser.parse_args()
    
    # Load events
    events = load_events(args.events_dir)
    
    if not events:
        print("No events found. Make sure the monitoring system has run.")
        return
    
    # If no specific option, show all
    if not any([args.summary, args.correlations, args.recent, 
                args.sessions, args.stats]):
        args.all = True
    
    # Display requested reports
    if args.summary or args.all:
        print_event_summary(events)
    
    if args.correlations or args.all:
        print_correlations(events)
    
    if args.recent:
        print_recent_events(events, args.recent)
    elif args.all:
        print_recent_events(events, hours=1)
    
    if args.sessions or args.all:
        print_person_sessions(events)
    
    if args.stats or args.all:
        print_sensor_stats(events)
    
    print("\n" + "="*80 + "\n")


if __name__ == "__main__":
    main()
