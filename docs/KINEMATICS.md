# Robotic Arm Kinematics — 4-DOF Pick & Place

## Physical Parameters

| Segment          | Length (approx.) | Joint        |
|------------------|------------------|--------------|
| Base → Shoulder  | 60 mm (height)   | Base Servo   |
| Shoulder → Elbow | 100 mm           | Shoulder Servo |
| Elbow → Wrist    | 90 mm            | Elbow Servo  |
| Wrist → Gripper  | 40 mm            | Gripper Servo |

Measure your actual robot and replace these values.

---

## Coordinate System

```
Z (up)
│
│    Y (forward)
│   /
│  /
└──────── X (right)
Origin = base servo center
```

---

## Forward Kinematics (FK)

Given joint angles → find end-effector (x, y, z):

```
θ_base     = base servo angle     (0–180°, 90° = front)
θ_shoulder = shoulder servo angle (0–180°, 90° = horizontal)
θ_elbow    = elbow servo angle    (0–180°)

L1 = 100 mm   (upper arm)
L2 =  90 mm   (forearm)
H0 =  60 mm   (base height offset)

r   = L1·cos(θ_shoulder) + L2·cos(θ_shoulder + θ_elbow - 180°)
z   = H0 + L1·sin(θ_shoulder) + L2·sin(θ_shoulder + θ_elbow - 180°)

x   = r · cos(θ_base - 90°)
y   = r · sin(θ_base - 90°)
```

---

## Inverse Kinematics (IK)

Given target (x, y, z) → find joint angles:

```python
import math

L1 = 100  # mm
L2 =  90  # mm
H0 =  60  # mm

def inverse_kinematics(x, y, z):
    # 1. Base angle (rotation around Z-axis)
    theta_base = math.degrees(math.atan2(y, x)) + 90  # offset to match servo 0–180

    # 2. Planar reach and height
    r = math.sqrt(x**2 + y**2)
    h = z - H0

    # 3. Check reachability
    D = math.sqrt(r**2 + h**2)
    if D > (L1 + L2):
        raise ValueError(f"Target out of reach: D={D:.1f} mm, max={L1+L2} mm")

    # 4. Elbow angle (law of cosines)
    cos_theta2 = (r**2 + h**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))   # clamp for floating point
    theta_elbow_rad = math.acos(cos_theta2)

    # 5. Shoulder angle
    alpha = math.atan2(h, r)
    beta  = math.atan2(L2 * math.sin(theta_elbow_rad), L1 + L2 * math.cos(theta_elbow_rad))
    theta_shoulder_rad = alpha - beta

    theta_shoulder = math.degrees(theta_shoulder_rad) + 90  # servo offset
    theta_elbow    = math.degrees(theta_elbow_rad) + 90     # servo offset (elbow up config)

    return (
        round(theta_base,     1),
        round(theta_shoulder, 1),
        round(theta_elbow,    1)
    )

# Example: pick object 120 mm in front, at ground level (z = 0)
base, shoulder, elbow = inverse_kinematics(120, 0, 0)
print(f"Base: {base}°  Shoulder: {shoulder}°  Elbow: {elbow}°")
```

---

## Pre-computed Pick & Place Waypoints

These match the demo sequence in the `.ino` file:

| Waypoint       | Base | Shoulder | Elbow | Gripper |
|----------------|------|----------|-------|---------|
| Home           | 90°  | 90°      | 90°   | Open    |
| Safe Travel    | 90°  | 100°     | 60°   | Open    |
| Pick Approach  | 90°  | 60°      | 120°  | Open    |
| Pick (grabbed) | 90°  | 60°      | 120°  | Closed  |
| Lift           | 90°  | 100°     | 60°   | Closed  |
| Place Position | 45°  | 65°      | 115°  | Closed  |
| Place (release)| 45°  | 65°      | 115°  | Open    |
| Return Home    | 90°  | 90°      | 90°   | Open    |

---

## Servo Angle Conventions

| Servo    | 0°          | 90°          | 180°          |
|----------|-------------|--------------|---------------|
| Base     | Full Left   | Centre/Front | Full Right    |
| Shoulder | Arm Down    | Horizontal   | Arm Up        |
| Elbow    | Fully bent  | Straight     | Over-extended |
| Gripper  | Fully Closed| Mid          | Fully Open    |

---

## Bluetooth Command Reference

| Key | Action            | Key | Action           |
|-----|-------------------|-----|------------------|
| F   | Drive Forward     | B   | Drive Backward   |
| L   | Turn Left         | R   | Turn Right       |
| S   | Stop              |     |                  |
| q   | Base Clockwise    | a   | Base Counter-CW  |
| w   | Shoulder Up       | s   | Shoulder Down    |
| e   | Elbow Up          | d   | Elbow Down       |
| o   | Gripper Open      | c   | Gripper Close    |
| P   | Auto Pick & Place | H   | Go Home          |

---

## Recommended Mobile App

Use **"Bluetooth RC Controller"** or **"Arduino Bluetooth Controller"** from the Play Store.
Map buttons to the characters above in the app settings.

---

## Calibration Tips

1. Upload code and send `H` to set home position first.
2. Manually adjust `BASE_HOME`, `SHOULDER_HOME`, `ELBOW_HOME` in the `.ino` if servos don't center correctly.
3. Measure your actual L1, L2 lengths and update the IK Python script.
4. Test the `P` (pick & place) command and tune waypoint angles for your object height.
