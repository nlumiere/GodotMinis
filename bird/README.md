# Bird Flight Simulator

3D bird flight physics demo in Godot 4 with C#.

## What It Does

- Simplified, Pseudorealistic bird flight with lift, drag, and angle of attack approximations
- Wing flapping animation with tucked wing mechanics
- Dual camera system (locked follow / free pan)
- Ground collision with stabilization
- Real-time HUD showing height above ground and airspeed

## Controls

**Keyboard:**
- WASD: Pitch/Roll
- Space: Flap wings
- Enter: Tuck wings (dive faster)
- Left/Right arrows: Pan camera
- Up arrow: Reset camera

**Gamepad:**
- Left stick: Pitch/Roll
- A: Flap wings
- B: Tuck wings
- Right stick: Pan camera
- Right stick click: Reset camera

## Technical Features

- **Physics**: Lift based on airspeed and angle of attack
- **Drag**: Differential drag system (forward/side/vertical)
- **Wings**: Animated flapping with position-based wing movement
- **Camera**: Smooth transitions between locked and free modes
- **Ground**: Impact friction and upright stabilization
- **HUD**: Raycast-based height measurement

## Files

- `Scripts/Bird.cs` - Main flight controller
- `Characters/bird.tscn` - Bird model and components
- `Demo.tscn` - Main scene with terrain
- `terrainmesh.*` - Terrain geometry

## How to Run

1. Open in Godot 4
2. Build C# project
3. Run `Demo.tscn`
