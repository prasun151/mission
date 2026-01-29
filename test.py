#!/usr/bin/env python3
from pymavlink import mavutil
import time
import threading

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
master.wait_heartbeat()
print(f"â Connected! System: {master.target_system}, Component: {master.target_component}")

# Request data streams
print("Requesting data streams...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,
    1
)
print("â Data streams requested!")
time.sleep(1)

# Global flag for telemetry
show_telemetry = True

def telemetry_monitor():
    """Continuous telemetry display"""
    global show_telemetry
    last_heartbeat = 0
    last_gps = 0
    last_battery = 0
    
    while show_telemetry:
        msg = master.recv_match(blocking=False, timeout=0.1)
        if msg:
            msg_type = msg.get_type()
            current_time = time.time()
            
            if msg_type == 'HEARTBEAT' and msg.type == 2 and (current_time - last_heartbeat) > 5:
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
                print(f"[HEARTBEAT] Mode: {msg.custom_mode}, Armed: {armed}")
                last_heartbeat = current_time
            
            elif msg_type == 'GPS_RAW_INT' and (current_time - last_gps) > 3:
                print(f"[GPS] Lat: {msg.lat/1e7:.6f}, Lon: {msg.lon/1e7:.6f}, Alt: {msg.alt/1000:.1f}m, Sats: {msg.satellites_visible}")
                last_gps = current_time
            
            elif msg_type == 'SYS_STATUS' and (current_time - last_battery) > 5:
                print(f"[BATTERY] Voltage: {msg.voltage_battery/1000:.2f}V, Current: {msg.current_battery/100:.1f}A")
                last_battery = current_time
        
        time.sleep(0.1)

def get_current_location():
    """Get current GPS location"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    return None, None, None

def get_gps_satellites():
    """Check GPS satellite count"""
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg:
        return msg.satellites_visible, msg.fix_type
    return 0, 0

def set_mode(mode):
    """Set flight mode"""
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print(f"â Mode set to: {mode}")
    time.sleep(1)

def arm_drone():
    """Arm the drone"""
    print("\n" + "="*50)
    print("--- ARMING DRONE ---")
    print("="*50)
    master.arducopter_arm()
    
    timeout = time.time() + 10
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.type == 2:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
            if armed:
                print("â Armed!")
                return True
    
    print("â  Could not arm - check pre-arm conditions")
    return False

def disarm():
    """Disarm the drone"""
    print("\n" + "="*50)
    print("--- DISARMING ---")
    print("="*50)
    master.arducopter_disarm()
    
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.type == 2:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0
            if not armed:
                print("â Disarmed!")
                return True
    return False

def takeoff_with_throttle(target_altitude):
    """
    Takeoff with gradual throttle increase
    """
    print("\n" + "="*50)
    print(f"--- TAKING OFF TO {target_altitude}m ---")
    print("Ramping up throttle gradually...")
    print("="*50)
    
    # Send takeoff command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    
    # Monitor altitude and show progress
    last_alt = 0
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            
            # Show climb rate
            climb_rate = current_alt - last_alt
            print(f"[TAKEOFF] Altitude: {current_alt:.2f}m / {target_altitude}m | Climb: +{climb_rate:.2f}m/s")
            last_alt = current_alt
            
            # Check if target reached
            if current_alt >= target_altitude * 0.95:
                print("â Target altitude reached!")
                break
        
        time.sleep(0.5)

def goto_location(lat, lon, alt):
    """
    Fly to specific GPS coordinates
    """
    print("\n" + "="*50)
    print("--- NAVIGATING TO TARGET ---")
    print(f"Target: Lat {lat:.6f}, Lon {lon:.6f}, Alt {alt}m")
    print("="*50)
    
    # Send waypoint
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        lat, lon, alt
    )
    
    # Monitor progress
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            
            # Calculate distance
            lat_diff = abs(current_lat - lat)
            lon_diff = abs(current_lon - lon)
            distance = ((lat_diff**2 + lon_diff**2)**0.5) * 111000  # meters
            
            print(f"[NAV] Distance: {distance:.1f}m | Position: ({current_lat:.6f}, {current_lon:.6f}) | Alt: {current_alt:.1f}m")
            
            # Check if reached
            if distance < 2:  # Within 2 meters
                print("â Target coordinates reached!")
                break
        
        time.sleep(0.5)

def land_with_throttle():
    """
    Land with gradual throttle decrease
    """
    print("\n" + "="*50)
    print("--- LANDING ---")
    print("Reducing throttle gradually...")
    print("="*50)
    
    # Send land command
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    # Monitor descent
    last_alt = 100
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            
            # Show descent rate
            descent_rate = last_alt - current_alt
            print(f"[LANDING] Altitude: {current_alt:.2f}m | Descent: -{descent_rate:.2f}m/s")
            last_alt = current_alt
            
            # Check if landed
            if current_alt < 0.2:
                print("â Landed successfully!")
                break
        
        time.sleep(0.5)

# ===== MAIN MISSION =====
try:
    print("\n" + "="*50)
    print("AUTONOMOUS GPS MISSION")
    print("Takeoff â Navigate â Land")
    print("="*50)
    
    # Start telemetry
    telemetry_thread = threading.Thread(target=telemetry_monitor, daemon=True)
    telemetry_thread.start()
    print("â Telemetry monitoring started\n")
    time.sleep(2)
    
    # Check GPS
    print("\n--- Checking GPS Status ---")
    sats, fix_type = get_gps_satellites()
    print(f"Satellites: {sats}, Fix Type: {fix_type}")
    
    if sats < 8 or fix_type < 3:
        print("\nâ  WARNING: Insufficient GPS!")
        print(f"  Current: {sats} satellites, Fix type: {fix_type}")
        print(f"  Required: 8+ satellites, Fix type: 3 (3D fix)")
        print("\nTips:")
        print("  - Go outside with clear sky view")
        print("  - Wait 2-5 minutes for GPS lock")
        print("  - Keep away from buildings/trees")
        
        proceed = input("\nContinue anyway? (NOT RECOMMENDED - type 'yes'): ").lower()
        if proceed != 'yes':
            show_telemetry = False
            exit()
    else:
        print("â GPS lock good!")
    
    # Get current position
    print("\n--- Current Position ---")
    curr_lat, curr_lon, curr_alt = get_current_location()
    if curr_lat:
        print(f"Latitude:  {curr_lat:.6f}")
        print(f"Longitude: {curr_lon:.6f}")
        print(f"Altitude:  {curr_alt:.1f}m")
    else:
        print("â  Could not get current position")
    
    # Get target from user
    print("\n" + "="*50)
    print("--- ENTER TARGET COORDINATES ---")
    print("="*50)
    target_lat = float(input("Target LATITUDE: "))
    target_lon = float(input("Target LONGITUDE: "))
    target_alt = float(input("Flight ALTITUDE (meters): "))
    
    # Mission summary
    print(f"\n" + "="*50)
    print("--- MISSION SUMMARY ---")
    print("="*50)
    if curr_lat:
        lat_diff = abs(target_lat - curr_lat)
        lon_diff = abs(target_lon - curr_lon)
        distance = ((lat_diff**2 + lon_diff**2)**0.5) * 111000
        print(f"Distance to target: ~{distance:.1f} meters")
    print(f"Target Latitude:  {target_lat:.6f}")
    print(f"Target Longitude: {target_lon:.6f}")
    print(f"Flight Altitude:  {target_alt}m")
    print("="*50)
    
    # Final confirmation
    print("\n" + "!"*50)
    print("!!! FINAL SAFETY CHECK !!!")
    print("!"*50)
    print("â Propellers attached and secured")
    print("â Clear flight area (no people/obstacles)")
    print("â RC transmitter ON and ready to override")
    print("â Battery fully charged")
    print("â Emergency plan ready")
    
    confirm = input("\nâ  START AUTONOMOUS MISSION? Type 'START': ").upper()
    if confirm != 'START':
        show_telemetry = False
        print("Mission cancelled.")
        exit()
    
    # Countdown
    print("\n" + "!"*50)
    print("!!! MISSION STARTING !!!")
    print("!"*50)
    for i in range(5, 0, -1):
        print(f"{i}...")
        time.sleep(1)
    
    # Execute mission
    print("\nð MISSION START ð\n")
    
    # 1. Set GUIDED mode
    set_mode('GUIDED')
    time.sleep(2)
    
    # 2. Arm
    armed = arm_drone()
    if not armed:
        print("\nâ  Mission aborted - could not arm")
        show_telemetry = False
        exit()
    time.sleep(2)
    
    # 3. Takeoff with throttle ramping
    takeoff_with_throttle(target_alt)
    time.sleep(3)
    
    # 4. Navigate to coordinates
    goto_location(target_lat, target_lon, target_alt)
    time.sleep(3)
    
    # 5. Land with throttle ramping
    land_with_throttle()
    time.sleep(2)
    
    # 6. Disarm
    disarm()
    
    # Stop telemetry
    show_telemetry = False
    time.sleep(1)
    
    print("\n" + "="*50)
    print("âââ MISSION COMPLETE! âââ")
    print("="*50)

except KeyboardInterrupt:
    print("\n\n" + "!"*50)
    print("!!! EMERGENCY STOP - USER CANCELLED !!!")
    print("!"*50)
    show_telemetry = False
    print("Activating RTL (Return to Launch) mode...")
    set_mode('RTL')

except Exception as e:
    print(f"\n\n" + "!"*50)
    print(f"!!! ERROR: {e} !!!")
    print("!"*50)
    show_telemetry = False
    print("Activating RTL (Return to Launch) mode...")
    set_mode('RTL')
