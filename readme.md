# OpenGL Soft Body Simulation

This project implements a soft body simulation using Extended Position Based Dynamics (XPBD) with OpenGL rendering. It features a real-time, interactive 3D environment where users can manipulate various parameters of the soft body simulation.
Using Mullers Papers
[Documentation]([https://docs.example.com](https://matthias-research.github.io/pages/publications/XPBD.pdf))


## Features

- Soft body simulation using XPBD
- Real-time rendering with OpenGL
- Shadow mapping for enhanced visual quality
- ImGui-based user interface for parameter adjustment
- Camera controls for scene navigation
- Support for loading custom 3D models


## Key Components

### SoftBodyXPBD Class

The `SoftBodyXPBD` class is the core of the soft body simulation. It includes:

- Particle system management
- Various constraints (distance, shearing, bending, volume)
- Shape matching for maintaining object structure
- Collision detection and response (ground constraint)
- Methods for updating particle positions and velocities


## Dependencies

- GLFW
- GLAD
- GLM
- ImGui
- stb_image (for texture loading)

## Building

This project uses CMake for build configuration. To build the project:

1. Ensure you have CMake installed
2. Clone this repository
3. Navigate to the project directory
4. Run the following commands:

mkdir build
cd build
cmake ..
make


## Usage

After building, run the executable generated in the `build` directory. Use the following controls:

- WASD: Move camera (when holding spacebar)
- Mouse: Look around (when 'E' is pressed to enable capture)
- E: Enable mouse capture
- Q: Disable mouse capture
- R: Reset soft body position
- Esc: Exit application

Use the ImGui window to adjust simulation parameters in real-time.

## Configuration

- Shader paths and model paths can be configured at the top of the main.cpp file
- Initial simulation parameters (like FPS and iterations) can be adjusted in the main function

## Contributing

Contributions to improve the simulation, add features, or optimize performance are welcome. Please submit a pull request with your changes.

