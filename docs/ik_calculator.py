"""
Inverse Kinematics Calculator
Robotic Arm Car — Pick & Place

Usage:
    python ik_calculator.py

Outputs servo angles for a given (x, y, z) target position.
"""

import math

# ── Arm dimensions (measure your actual robot!) ──────────────
L1 = 100   # mm  upper arm (shoulder → elbow)
L2 =  90   # mm  forearm   (elbow → gripper tip)
H0 =  60   # mm  base height (ground → shoulder pivot)


def forward_kinematics(theta_base_deg, theta_shoulder_deg, theta_elbow_deg):
    """
    Returns (x, y, z) end-effector position in mm.
    All angles in servo degrees (0–180°).
    """
    tb = math.radians(theta_base_deg - 90)      # offset: 90° = forward
    ts = math.radians(theta_shoulder_deg - 90)  # offset: 90° = horizontal
    te = math.radians(theta_elbow_deg - 90)     # offset: 90° = straight

    r = L1 * math.cos(ts) + L2 * math.cos(ts + te)
    z = H0 + L1 * math.sin(ts) + L2 * math.sin(ts + te)

    x = r * math.cos(tb)
    y = r * math.sin(tb)

    return round(x, 1), round(y, 1), round(z, 1)


def inverse_kinematics(x, y, z):
    """
    Returns (base°, shoulder°, elbow°) servo angles for target (x, y, z) mm.
    Raises ValueError if target is out of reach.
    """
    # 1. Base rotation
    theta_base = math.degrees(math.atan2(y, x)) + 90
    theta_base = max(0, min(180, theta_base))

    # 2. Planar distance and height
    r = math.sqrt(x**2 + y**2)
    h = z - H0

    # 3. Reachability check
    D = math.sqrt(r**2 + h**2)
    max_reach = L1 + L2
    if D > max_reach:
        raise ValueError(
            f"Target ({x}, {y}, {z}) is out of reach.\n"
            f"Distance: {D:.1f} mm  Max reach: {max_reach} mm"
        )

    # 4. Elbow angle (law of cosines)
    cos_e = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_e = max(-1.0, min(1.0, cos_e))
    elbow_rad = math.acos(cos_e)

    # 5. Shoulder angle
    alpha = math.atan2(h, r)
    beta  = math.atan2(L2 * math.sin(elbow_rad), L1 + L2 * math.cos(elbow_rad))
    shoulder_rad = alpha - beta

    theta_shoulder = math.degrees(shoulder_rad) + 90
    theta_elbow    = math.degrees(elbow_rad)    + 90

    # Clamp to servo limits
    theta_base     = max(0,   min(180, theta_base))
    theta_shoulder = max(30,  min(150, theta_shoulder))
    theta_elbow    = max(0,   min(150, theta_elbow))

    return round(theta_base, 1), round(theta_shoulder, 1), round(theta_elbow, 1)


def print_waypoints():
    """Print the pre-computed pick & place waypoint table."""
    print("\n── Pick & Place Waypoints ──────────────────")
    waypoints = [
        ("Home",          90, 90,  90,  "Open"),
        ("Safe Travel",   90, 100, 60,  "Open"),
        ("Pick Approach", 90, 60,  120, "Open"),
        ("PICK",          90, 60,  120, "CLOSED"),
        ("Lift",          90, 100, 60,  "Closed"),
        ("Place Pos",     45, 65,  115, "Closed"),
        ("PLACE",         45, 65,  115, "OPEN"),
        ("Home",          90, 90,  90,  "Open"),
    ]
    print(f"{'Waypoint':<16} {'Base':>6} {'Shoulder':>10} {'Elbow':>7} {'Gripper':>10}")
    print("-" * 55)
    for name, b, s, e, g in waypoints:
        fk = forward_kinematics(b, s, e)
        print(f"{name:<16} {b:>5}°  {s:>8}°  {e:>6}°  {g:>10}   → FK: {fk}")


if __name__ == "__main__":
    print("=" * 55)
    print("  Robotic Arm IK Calculator")
    print(f"  L1={L1}mm  L2={L2}mm  H0={H0}mm")
    print("=" * 55)

    print_waypoints()

    print("\n── Custom Target Solver ────────────────────")
    targets = [
        (120,   0,  0),    # straight ahead at ground
        (100,  60,  0),    # 30° to the side
        ( 80,   0, 50),    # raised target
    ]

    for tx, ty, tz in targets:
        try:
            b, s, e = inverse_kinematics(tx, ty, tz)
            print(f"  Target ({tx:4}, {ty:3}, {tz:3}) mm → Base:{b}° Shoulder:{s}° Elbow:{e}°")
        except ValueError as err:
            print(f"  ✗ {err}")

    print()
