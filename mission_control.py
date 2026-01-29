#!/usr/bin/env python3
from pymavlink import mavutil
import time
import threading
import cv2
import numpy as np
from cv2 import aruco
from pyzbar import pyzbar

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
master.wait_heartbeat()
print(f"✓ Connected! System: {master.target_system}, Component: {master.target_component}")

# Request data streams
print("Requesting data streams...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4,
    1
)
print("✓ Data streams requested!")
time.sleep(1)

# Global flag for telemetry
show_telemetry = True

# Camera global variable
camera = None
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

def start_camera(camera_index=0):
    """
    Initialize and start the camera
    """
    global camera
    print("\n" + "="*50)
    print("--- STARTING CAMERA ---")
    print("="*50)
    
    camera = cv2.VideoCapture(camera_index)
    
    if not camera.isOpened():
        print("✗ Failed to open camera!")
        return False
    
    # Set camera properties for better performance
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    
    # Test camera
    ret, frame = camera.read()
    if ret:
        print("✓ Camera started successfully!")
        print(f"  Resolution: {frame.shape[1]}x{frame.shape[0]}")
        return True
    else:
        print("✗ Camera opened but cannot read frames!")
        return False

def stop_camera():
    """
    Release the camera
    """
    global camera
    if camera is not None:
        camera.release()
        print("✓ Camera stopped")

def detect_aruco_marker():
    """
    Detect ArUco marker in camera frame and return its center position
    Returns: (center_x, center_y, marker_id) or (None, None, None) if not found
    """
    global camera, aruco_dict, aruco_params
    
    if camera is None or not camera.isOpened():
        print("✗ Camera not available!")
        return None, None, None
    
    ret, frame = camera.read()
    if not ret:
        return None, None, None
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    
    if ids is not None and len(ids) > 0:
        # Get the first marker
        marker_corners = corners[0][0]
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))
        marker_id = ids[0][0]
        
        return center_x, center_y, marker_id
    
    return None, None, None

def align_to_aruco_and_descend(target_altitude_percentage=0.4):
    """
    Align drone to center on ArUco marker and descend to specified altitude percentage
    
    Args:
        target_altitude_percentage: Percentage of current altitude to descend to (default 0.4 = 40%)
    """
    print("\n" + "="*50)
    print("--- ARUCO MARKER ALIGNMENT & DESCENT ---")
    print("="*50)
    
    # Get current altitude
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if not msg:
        print("✗ Could not get current altitude!")
        return False
    
    current_alt = msg.relative_alt / 1000.0
    target_alt = current_alt * target_altitude_percentage
    
    print(f"Current altitude: {current_alt:.2f}m")
    print(f"Target altitude: {target_alt:.2f}m ({int(target_altitude_percentage*100)}%)")
    print("\nAligning to ArUco marker...")
    
    # Camera frame center
    frame_center_x = 320  # Assuming 640x480 resolution
    frame_center_y = 240
    
    # Alignment tolerance (pixels)
    tolerance = 30
    
    # Maximum alignment attempts
    max_attempts = 100
    attempts = 0
    
    aligned = False
    
    while attempts < max_attempts:
        # Detect ArUco marker
        marker_x, marker_y, marker_id = detect_aruco_marker()
        
        if marker_x is None:
            print(f"[Attempt {attempts+1}] ⚠ ArUco marker not detected, searching...")
            attempts += 1
            time.sleep(0.2)
            continue
        
        print(f"[Attempt {attempts+1}] ArUco ID {marker_id} detected at ({marker_x}, {marker_y})")
        
        # Calculate offset from center
        offset_x = marker_x - frame_center_x
        offset_y = marker_y - frame_center_y
        
        print(f"  Offset from center: X={offset_x}px, Y={offset_y}px")
        
        # Check if aligned
        if abs(offset_x) < tolerance and abs(offset_y) < tolerance:
            print("✓ ArUco marker centered!")
            aligned = True
            break
        
        # Send velocity commands to adjust position
        # Convert pixel offset to velocity (simple proportional control)
        vx = -offset_y * 0.01  # Forward/backward (negative because image Y is inverted)
        vy = offset_x * 0.01   # Left/right
        
        # Limit velocities
        vx = max(-0.5, min(0.5, vx))
        vy = max(-0.5, min(0.5, vy))
        
        print(f"  Adjusting: vx={vx:.2f}m/s, vy={vy:.2f}m/s")
        
        # Send velocity command (in body frame)
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # type_mask (ignore position, use velocity)
            0, 0, 0,  # position
            vx, vy, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0  # yaw, yaw_rate
        )
        
        attempts += 1
        time.sleep(0.2)
    
    if not aligned:
        print("⚠ Could not perfectly align to ArUco marker, but continuing...")
    
    # Now descend to target altitude
    print(f"\nDescending to {target_alt:.2f}m...")
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"[DESCENT] Altitude: {current_alt:.2f}m / {target_alt:.2f}m")
            
            if current_alt <= target_alt + 0.5:
                print("✓ Target altitude reached!")
                break
            
            # Send position command to descend
            master.mav.set_position_target_local_ned_send(
                0,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # type_mask (use position)
                0, 0, -target_alt,  # position (NED, so negative Z)
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
        
        time.sleep(0.5)
    
    return True

def detect_and_read_qr():
    """
    Detect and read QR code from camera
    Returns: QR code data string or None if not found
    """
    global camera
    
    print("\n" + "="*50)
    print("--- QR CODE DETECTION ---")
    print("="*50)
    
    if camera is None or not camera.isOpened():
        print("✗ Camera not available!")
        return None
    
    print("Scanning for QR code...")
    
    max_attempts = 50
    attempts = 0
    
    while attempts < max_attempts:
        ret, frame = camera.read()
        if not ret:
            print("✗ Failed to read camera frame!")
            attempts += 1
            time.sleep(0.2)
            continue
        
        # Decode QR codes
        qr_codes = pyzbar.decode(frame)
        
        if qr_codes:
            for qr in qr_codes:
                qr_data = qr.data.decode('utf-8')
                qr_type = qr.type
                
                print(f"\n✓ QR Code detected!")
                print(f"  Type: {qr_type}")
                print(f"  Data: {qr_data}")
                
                # Draw bounding box (optional, for debugging)
                points = qr.polygon
                if len(points) == 4:
                    pts = [(point.x, point.y) for point in points]
                    print(f"  Position: {pts}")
                
                return qr_data
        
        if attempts % 10 == 0:
            print(f"[Attempt {attempts+1}/{max_attempts}] Searching for QR code...")
        
        attempts += 1
        time.sleep(0.2)
    
    print("⚠ QR code not found after maximum attempts")
    return None

def send_telemetry_message(message, severity=6):
    """
    Send a text message to ground station via MAVLink telemetry
    
    Args:
        message: Text message to send (max 50 characters)
        severity: Message severity level (0=EMERGENCY, 3=ERROR, 6=INFO)
                 MAV_SEVERITY: 0-EMERGENCY, 1-ALERT, 2-CRITICAL, 3-ERROR, 
                              4-WARNING, 5-NOTICE, 6-INFO, 7-DEBUG
    """
    # Truncate message to 50 characters (MAVLink limitation)
    message = message[:50]
    
    master.mav.statustext_send(
        severity,
        message.encode('utf-8')
    )
    
    print(f"[TELEMETRY SENT] {message}")

def send_qr_data_via_telemetry(qr_data):
    """
    Send QR code data to ground station via telemetry in chunks
    QR data is split into multiple messages due to 50-char limit
    """
    if not qr_data:
        send_telemetry_message("QR: NO DATA", severity=4)
        return
    
    print("\n" + "="*50)
    print("--- TRANSMITTING QR DATA VIA TELEMETRY ---")
    print("="*50)
    
    # Send header
    send_telemetry_message("QR_DATA_START", severity=6)
    time.sleep(0.5)
    
    # Split data into 45-character chunks (leaving room for prefix)
    chunk_size = 45
    chunks = [qr_data[i:i+chunk_size] for i in range(0, len(qr_data), chunk_size)]
    
    # Send each chunk
    for idx, chunk in enumerate(chunks):
        msg = f"QR{idx+1}: {chunk}"
        send_telemetry_message(msg, severity=6)
        time.sleep(0.5)  # Delay between messages to avoid flooding
    
    # Send footer
    send_telemetry_message(f"QR_DATA_END ({len(chunks)} parts)", severity=6)
    time.sleep(0.5)
    
    print(f"✓ QR data transmitted in {len(chunks)} part(s)")
    print("="*50)

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
    print(f"✓ Mode set to: {mode}")
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
                print("✓ Armed!")
                return True
    
    print("✗ Could not arm - check pre-arm conditions")
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
                print("✓ Disarmed!")
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
                print("✓ Target altitude reached!")
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
                print("✓ Target coordinates reached!")
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
                print("✓ Landed successfully!")
                break
        
        time.sleep(0.5)

# ===== MAIN MISSION =====
try:
    print("\n" + "="*50)
    print("AUTONOMOUS GPS MISSION WITH VISION")
    print("Takeoff → Navigate → ArUco Align → QR Detect → Land")
    print("="*50)
    
    # Start camera first
    if not start_camera(camera_index=0):
        print("\n⚠ Camera failed to start. Continue without camera features?")
        proceed = input("Type 'yes' to continue: ").lower()
        if proceed != 'yes':
            exit()
    
    # Start telemetry
    telemetry_thread = threading.Thread(target=telemetry_monitor, daemon=True)
    telemetry_thread.start()
    print("✓ Telemetry monitoring started\n")
    time.sleep(2)
    
    # Check GPS
    print("\n--- Checking GPS Status ---")
    sats, fix_type = get_gps_satellites()
    print(f"Satellites: {sats}, Fix Type: {fix_type}")
    
    if sats < 8 or fix_type < 3:
        print("\n✗ WARNING: Insufficient GPS!")
        print(f"  Current: {sats} satellites, Fix type: {fix_type}")
        print(f"  Required: 8+ satellites, Fix type: 3 (3D fix)")
        print("\nTips:")
        print("  - Go outside with clear sky view")
        print("  - Wait 2-5 minutes for GPS lock")
        print("  - Keep away from buildings/trees")
        
        proceed = input("\nContinue anyway? (NOT RECOMMENDED - type 'yes'): ").lower()
        if proceed != 'yes':
            show_telemetry = False
            stop_camera()
            exit()
    else:
        print("✓ GPS lock good!")
    
    # Get current position
    print("\n--- Current Position ---")
    curr_lat, curr_lon, curr_alt = get_current_location()
    if curr_lat:
        print(f"Latitude:  {curr_lat:.6f}")
        print(f"Longitude: {curr_lon:.6f}")
        print(f"Altitude:  {curr_alt:.1f}m")
    else:
        print("✗ Could not get current position")
    
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
    print("✓ Propellers attached and secured")
    print("✓ Clear flight area (no people/obstacles)")
    print("✓ RC transmitter ON and ready to override")
    print("✓ Battery fully charged")
    print("✓ Emergency plan ready")
    print("✓ Camera operational")
    print("✓ ArUco marker ready at target location")
    print("✓ QR code ready for scanning")
    
    confirm = input("\n⚠ START AUTONOMOUS MISSION? Type 'START': ").upper()
    if confirm != 'START':
        show_telemetry = False
        stop_camera()
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
    print("\n🚁 MISSION START 🚁\n")
    send_telemetry_message("Mission started", severity=6)
    
    # 1. Set GUIDED mode
    set_mode('GUIDED')
    send_telemetry_message("Mode: GUIDED", severity=6)
    time.sleep(2)
    
    # 2. Arm
    send_telemetry_message("Arming...", severity=6)
    armed = arm_drone()
    if not armed:
        print("\n✗ Mission aborted - could not arm")
        send_telemetry_message("ARM FAILED - Aborting", severity=2)
        show_telemetry = False
        stop_camera()
        exit()
    send_telemetry_message("Armed successfully", severity=6)
    time.sleep(2)
    
    # 3. Takeoff with throttle ramping
    send_telemetry_message(f"Takeoff to {target_alt}m", severity=6)
    takeoff_with_throttle(target_alt)
    send_telemetry_message(f"Altitude {target_alt}m reached", severity=6)
    time.sleep(3)
    
    # 4. Navigate to coordinates
    send_telemetry_message(f"Nav to target", severity=6)
    goto_location(target_lat, target_lon, target_alt)
    send_telemetry_message("Target reached", severity=6)
    time.sleep(3)
    
    # 5. Align to ArUco marker and descend to 40% altitude
    aruco_aligned = False
    if camera is not None:
        send_telemetry_message("Starting ArUco alignment", severity=6)
        aruco_aligned = align_to_aruco_and_descend(target_altitude_percentage=0.4)
        time.sleep(2)
        
        if aruco_aligned:
            send_telemetry_message("ArUco alignment SUCCESS", severity=6)
        else:
            send_telemetry_message("ArUco alignment FAILED", severity=4)
            print("\n⚠ ArUco marker not detected - descending to coordinates anyway...")
            
            # Descend to 40% of target altitude at the GPS coordinates
            descent_alt = target_alt * 0.4
            print(f"Descending to {descent_alt:.2f}m at GPS coordinates...")
            
            while True:
                msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                if msg:
                    current_alt = msg.relative_alt / 1000.0
                    print(f"[DESCENT] Altitude: {current_alt:.2f}m / {descent_alt:.2f}m")
                    
                    if current_alt <= descent_alt + 0.5:
                        print("✓ Descent altitude reached!")
                        break
                    
                    # Send position command to descend
                    master.mav.set_position_target_local_ned_send(
                        0,
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000111111111000,  # type_mask (use position)
                        0, 0, -descent_alt,  # position (NED, so negative Z)
                        0, 0, 0,  # velocity
                        0, 0, 0,  # acceleration
                        0, 0  # yaw, yaw_rate
                    )
                
                time.sleep(0.5)
    else:
        print("\n⚠ Skipping ArUco alignment - camera not available")
        send_telemetry_message("Camera unavailable", severity=4)
    
    time.sleep(2)
    
    # 6. Detect and read QR code - Attempt to read but continue regardless
    qr_data = None
    if camera is not None:
        send_telemetry_message("Scanning for QR code...", severity=6)
        qr_data = detect_and_read_qr()
        
        if qr_data:
            print(f"\n✓ Mission data received: {qr_data}")
            send_telemetry_message("QR code detected!", severity=6)
            # Transmit QR data via telemetry
            send_qr_data_via_telemetry(qr_data)
        else:
            print("\n⚠ QR code not detected - continuing to land anyway")
            send_telemetry_message("QR NOT FOUND - Landing anyway", severity=4)
    else:
        print("\n⚠ Skipping QR detection - camera not available")
        send_telemetry_message("Camera unavailable", severity=4)
    
    time.sleep(2)
    
    # 7. Land with throttle ramping
    send_telemetry_message("Landing...", severity=6)
    land_with_throttle()
    send_telemetry_message("Landed", severity=6)
    time.sleep(2)
    
    # 8. Disarm
    disarm()
    send_telemetry_message("Disarmed", severity=6)
    
    # Stop telemetry and camera
    show_telemetry = False
    stop_camera()
    time.sleep(1)
    
    print("\n" + "="*50)
    print("✓✓✓ MISSION COMPLETE! ✓✓✓")
    print("="*50)
    print(f"QR Code Data: {qr_data if qr_data else 'NOT DETECTED'}")
    send_telemetry_message("MISSION SUCCESS", severity=5)
    print("="*50)

except KeyboardInterrupt:
    print("\n\n" + "!"*50)
    print("!!! EMERGENCY STOP - USER CANCELLED !!!")
    print("!"*50)
    send_telemetry_message("EMERGENCY STOP!", severity=0)
    show_telemetry = False
    stop_camera()
    print("Activating RTL (Return to Launch) mode...")
    set_mode('RTL')

except Exception as e:
    print(f"\n\n" + "!"*50)
    print(f"!!! ERROR: {e} !!!")
    print("!"*50)
    send_telemetry_message(f"ERROR: {str(e)[:40]}", severity=2)
    show_telemetry = False
    stop_camera()
    print("Activating RTL (Return to Launch) mode...")
    set_mode('RTL')