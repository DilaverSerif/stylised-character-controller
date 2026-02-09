# Stylised Character Controller

A **stylised physics-based character controller** for Unity 3D, featuring smooth movement, dynamic animations, and a comprehensive oscillator system. Built using Unity's Universal Render Pipeline (URP) and inspired by the floating capsule approach from Toyful Games' Very Very Valet.

[![Unity Version](https://img.shields.io/badge/Unity-6000.3.5f2-blue.svg)](https://unity3d.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![URP](https://img.shields.io/badge/URP-17.3.0-lightgrey.svg)](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)

## 🎮 Try It Out

Before diving into the technical details, get a hands-on feel for the project:

- **[Play on itch.io](https://joebinns.itch.io/stylised-character-controller)** - Interactive web demo
- **[Watch the Demo Video](https://youtu.be/3GsXkzbfNBo)** - See the controller in action
- **[Oscillators Explained](https://youtu.be/0gWJDWCvLUY)** - Deep dive into the oscillator system

[<img alt="Stylised Character Controller: Demo" width="503" src="https://joebinns.com/documents/fake_thumbnails/stylised_character_controller_thumbnail_time.png" />](https://youtu.be/3GsXkzbfNBo)
[<img alt="Oscillators for Game Development" width="503" src="https://joebinns.com/documents/fake_thumbnails/the_joy_of_oscillators_thumbnail_time.png" />](https://youtu.be/0gWJDWCvLUY)

## 📖 Overview

This character controller implements a **floating capsule physics-based approach** originally devised by [Toyful Games](https://www.toyfulgames.com/) for their game [Very Very Valet](https://www.toyfulgames.com/very-very-valet). The system uses spring-damper mechanics to create smooth, responsive character movement that feels natural and satisfying.

The controller is built as a fan-made independent recreation based on the techniques outlined in [Toyful Games' development video](https://www.youtube.com/watch?v=qdskE8PJy6Q&ab_channel=ToyfulGames), which provides code snippets and explanations of the movement system.

**Additional stylisation features** inspired by Toyful Games' blog posts on [character animations](https://www.toyfulgames.com/blog/character-animations) and [shaders and effects](https://www.toyfulgames.com/blog/deep-dive-shaders-and-effects) enhance the visual appeal and gameplay feel.

## ✨ Features

### Core Character Controller
- **Physics-based movement** using floating capsule approach with spring-damper mechanics
- **Smooth ground detection** with configurable ride height and ground snapping
- **Advanced air control** with customizable multiplier for mid-air movement
- **Slope handling** with maximum slope angle detection and prevention
- **Coyote time** for more forgiving jump mechanics
- **Unity Events system** for easy integration with other systems (OnJump, OnLand, OnGrounded, etc.)

### Visual Effects
- **Squash and stretch** animations that make motion appear more fluid and bouncy
- **Dithered silhouettes** that appear when the character is obscured, ensuring visibility at all times
- **Top-down blob shadows** for sharper 3D platforming feel, complementing Unity's built-in shadows
- **Dust particles** that spawn during movement, adding life to character motion

### Development Tools
- **Editor visualizer** (`PhysicsBasedCharacterControllerVisualizer`) for scene visualization and runtime debugging in the Unity Editor
- **Comprehensive gizmo system** for oscillator visualization (toggle with F1)

### Oscillator System
- **Oscillator** - Damped oscillator using transform local position with configurable stiffness, damper, and mass
- **Torsional Oscillator** - Rotational oscillator using rigidbody torque for dynamic rotational motion
- **Integration with Path Creator** - Oscillators work seamlessly with Sebastian Lague's Path Creator for moving platforms

## 🏗️ Technical Details

### Architecture

The character controller uses a **floating capsule** approach where:

1. **Ground Detection**: A raycast determines the distance to the ground and calculates the ground normal
2. **Spring-Damper System**: Spring forces maintain the desired ride height above the ground surface
3. **Movement Forces**: Acceleration-based movement applies forces to reach target velocities
4. **Upright Orientation**: Additional spring-damper system keeps the character upright relative to ground normal
5. **Jump Mechanics**: Impulse-based jumping with coyote time and jump state management

### Key Components

- **`PhysicsBasedCharacterController`** - Main controller component handling all physics calculations
- **`CharacterControllerInput`** - Input handler using Unity's Input System
- **`PhysicsBasedCharacterControllerVisualizer`** - Editor-only visualization tool
- **`Oscillator`** - Position-based oscillator for dynamic objects
- **`TorsionalOscillator`** - Rotation-based oscillator for rotational motion

### Physics Parameters

The controller exposes numerous configurable parameters:

- **Height Spring**: `rideHeight`, `rideSpringStrength`, `rideSpringDamper`
- **Upright Spring**: `uprightSpringStrength`, `uprightSpringDamper`
- **Movement**: `maxSpeed`, `maxAccel`, `maxAirAccel`, `airControlMultiplier`
- **Jump**: `jumpForce`, `coyoteTime`, `jumpBufferTime`
- **Advanced**: `maxSlopeAngle`, `groundSnapDistance`, `rayToGroundLength`

## 📦 Requirements

- **Unity Version**: 6000.3.5f2 (Unity 6) or compatible
- **Render Pipeline**: Universal Render Pipeline (URP) 17.3.0
- **Input System**: Unity Input System package (included)

## 🚀 Installation

1. **Clone or download** this repository
2. **Open the project** in Unity 6 (or compatible version)
3. **Wait for Unity** to import all assets and compile scripts
4. **Open the Demo scene** located at `Assets/Scenes/Demo.unity`
5. **Press Play** to start testing the character controller

### Project Structure

```
Assets/
├── Scripts/
│   ├── Physics Based Character Controller/
│   │   ├── PhysicsBasedCharacterController.cs      # Main controller
│   │   ├── PhysicsBasedCharacterControllerVisualizer.cs  # Editor visualizer
│   │   ├── CharacterControllerInput.cs            # Input handling
│   │   ├── RigidParent.cs                         # Moving platform support
│   │   └── RigidPlatform.cs                       # Platform component
│   ├── Oscillator/
│   │   ├── Oscillator.cs                          # Position oscillator
│   │   ├── TorsionalOscillator.cs                 # Rotation oscillator
│   │   ├── SquashAndStretch.cs                    # Animation effect
│   │   └── [Additional oscillator utilities]
│   ├── Audio/                                      # Audio management system
│   ├── Camera/                                     # Camera controllers
│   ├── Utilities/                                  # Helper utilities
│   └── Presentations/                              # Presentation system
└── Scenes/
    ├── Demo.unity                                  # Main demo scene
    └── Oscillator Demo.unity                       # Oscillator showcase
```

## 🎮 Usage

### Basic Controls

When running the project in the Game view:
- **<kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>** - Move character
- **<kbd>Space</kbd>** - Jump
- **<kbd>F1</kbd>** - Toggle oscillator gizmos

### Using the Character Controller

#### Basic Setup

1. Add a `Rigidbody` component to your character GameObject
2. Add the `PhysicsBasedCharacterController` component
3. Configure the controller parameters in the Inspector
4. Optionally add `CharacterControllerInput` for automatic input handling

#### Code Example

```csharp
using UnityEngine;

public class MyCharacterController : MonoBehaviour
{
    private PhysicsBasedCharacterController controller;
    
    void Start()
    {
        controller = GetComponent<PhysicsBasedCharacterController>();
        
        // Subscribe to events
        controller.OnJump.AddListener(HandleJump);
        controller.OnLand.AddListener(HandleLand);
    }
    
    void Update()
    {
        // Set movement input
        Vector3 moveInput = new Vector3(
            Input.GetAxis("Horizontal"),
            0f,
            Input.GetAxis("Vertical")
        );
        controller.SetMoveInput(moveInput);
        
        // Set jump input
        if (Input.GetButtonDown("Jump"))
        {
            controller.SetJumpInput(Vector3.up, jumpStarted: true);
        }
    }
    
    void HandleJump()
    {
        Debug.Log("Character jumped!");
    }
    
    void HandleLand()
    {
        Debug.Log("Character landed!");
    }
}
```

#### API Reference

**Public Methods:**
- `void SetMoveInput(Vector3 moveInput)` - Set movement input direction
- `void SetJumpInput(Vector3 jumpInput, bool jumpStarted = false)` - Set jump input

**Public Properties:**
- `bool IsGrounded` - Whether the character is currently grounded
- `Vector3 Velocity` - Current linear velocity
- `Vector3 GoalVelocity` - Current target velocity for movement

**Unity Events:**
- `OnJump` - Invoked when the character jumps
- `OnLand` - Invoked when the character lands
- `OnGrounded` - Invoked every frame while grounded
- `OnUngrounded` - Invoked every frame while not grounded
- `OnMoveStarted` - Invoked when movement begins
- `OnMoveStopped` - Invoked when movement stops

### Using the Editor Visualizer

The `PhysicsBasedCharacterControllerVisualizer` provides real-time debugging information in the Scene view:

1. Add the component to a GameObject with `PhysicsBasedCharacterController`
2. Enable "Show Visualization" in the Inspector
3. Configure visualization settings:
   - **Show Basic Info** - Display ground distance, velocity, etc.
   - **Show Advanced Info** - Display spring forces, ground angle, etc.
4. View visualizations in the Scene view during Play mode

### Using Oscillators

#### Basic Oscillator

```csharp
// Add Oscillator component to a GameObject
// Configure in Inspector:
// - localEquilibriumPosition: Rest position
// - stiffness: Spring strength (higher = stiffer)
// - damper: Damping factor (higher = faster decay)
// - mass: Oscillator mass
```

#### Torsional Oscillator

```csharp
// Add TorsionalOscillator component to a GameObject with Rigidbody
// Configure in Inspector:
// - localEquilibriumRotation: Rest rotation
// - stiffness: Rotational spring strength
// - damper: Rotational damping
// - localPivotPosition: Center of rotation
```

## 🔧 Troubleshooting

### Character Falls Through Ground
- Ensure ground objects are on the correct layer (check `_terrainLayer` mask)
- Verify `rayToGroundLength` is greater than `rideHeight`
- Check that ground colliders are not set as triggers

### Character Feels Sluggish
- Increase `maxAccel` for faster acceleration
- Adjust `rideSpringStrength` for more responsive height control
- Check `maxSpeed` is not too low

### Jump Not Working
- Verify `jumpForce` is set to a reasonable value (try 10-20)
- Check that `coyoteTime` and `jumpBufferTime` are configured
- Ensure ground detection is working (check `IsGrounded` property)

### Editor Visualizer Not Showing
- Ensure "Show Visualization" is enabled in the Inspector
- Check that you're viewing the Scene view (not Game view)
- Verify the component is attached to a GameObject with `PhysicsBasedCharacterController`

### Performance Issues
- Reduce `rayToGroundLength` if it's unnecessarily long
- Limit the number of active oscillators
- Consider reducing physics update frequency for non-critical objects

## 🤝 Contributing

Contributions are welcome and encouraged! Here's how you can help:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/my-awesome-feature`
3. **Make your changes** following the existing code style
4. **Test your changes** thoroughly
5. **Commit your changes**: `git commit -am 'Add awesome feature'`
6. **Push to your branch**: `git push origin feature/my-awesome-feature`
7. **Submit a pull request**

### Code Style Guidelines

- Follow Unity C# naming conventions (PascalCase for public members, camelCase for private)
- Add XML documentation comments for public APIs
- Use meaningful variable and method names
- Keep methods focused and single-purpose
- Add comments for complex logic

All contributions, big and small, are appreciated!

## 📚 Resources

- [Toyful Games - Very Very Valet](https://www.toyfulgames.com/very-very-valet)
- [Character Controller Development Video](https://www.youtube.com/watch?v=qdskE8PJy6Q&ab_channel=ToyfulGames)
- [Character Animations Blog Post](https://www.toyfulgames.com/blog/character-animations)
- [Shaders and Effects Blog Post](https://www.toyfulgames.com/blog/deep-dive-shaders-and-effects)
- [Unity Universal Render Pipeline Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)

## 🙏 Credits

### Physics Based Character Controller [^1]

The core character controller system is based on the work of [Toyful Games](https://www.toyfulgames.com/). A significant portion of this system's implementation follows the techniques and code snippets presented in their [development video](https://www.youtube.com/watch?v=qdskE8PJy6Q&ab_channel=ToyfulGames). This project is a fan-made independent recreation and is not officially affiliated with Toyful Games.

### Blob Shadows

The blob shadow system uses [Nyahoon Games'](http://nyahoon.com/products) asset [Dynamic Shadow Projector for URP](http://nyahoon.com/products/dynamic-shadow-projector).

### Path Creator

Moving platforms utilize Sebastian Lague's excellent [Path Creator](https://github.com/SebLague/Path-Creator) tool, which has been adapted to work with the `Oscillator` system.

### Additional Assets

- **Popcron Gizmos** - Runtime gizmo rendering (via Git package)
- **Unity Input System** - Modern input handling
- **Cinemachine** - Camera system (included in demo)

### Everything Else

All other code, shaders, and implementations are original work by [joebinns](https://joebinns.com/).

[^1]: **Disclaimer:** This project is fan-made! The quality of this project does not reflect the quality of Toyful Games or their products.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

**Made with ❤️ for the Unity community**
