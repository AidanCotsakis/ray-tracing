# Ray Tracing Renderer

A ray tracing renderer in Python that simulates realistic light behavior with configurable scene elements. This project uses Pygame and NumPy to render frames with Monte Carlo-based ray tracing for realistic reflections and light diffusion.

## Features

- **Ray Tracing with Recursive Reflections**: Simulates light paths through recursive reflections, enabling photorealistic effects.
- **Sphere and Triangle Collisions**: Detects ray intersections with both spheres and triangles, allowing complex 3D environments.
- **Randomized Reflections for Realism**: Uses Monte Carlo integration to create randomized light scattering for natural diffusion.
- **Dynamic Rendering**: Draws each frame pixel-by-pixel and saves outputs as images.

## Setup

1. **Clone the repository**.
2. **Install dependencies**:
    ```bash
    pip install pygame numpy
    ```

## **Usage**
Run the program to start rendering.
```bash
python rayTracing.py
```
Each frame will be saved as an image in the `output` folder.

## **Code Structure**
- `cameraObj`: Sets the position, field of view, and pixel resolution of the virtual camera.
- `sphereObj` and triangleObj: Define geometric objects for the scene, including color, emission, and reflectivity.
- `calculateRayPath`: Calculates light paths through the scene, determining intersections and reflections.
- `draw`: Manages rendering of each frame, pixel-by-pixel, and handles quit events.

## **Customization**
- **Object Properties**: Customize sphere and triangle properties like color, size, and reflectivity.
- **Camera Settings**: Adjust camera position, field of view, and resolution to alter scene perspective.
- **Rendering Passes**: Modify `maxBounces` to control the depth of ray recursion for different visual effects.

## Examples
- **Basic Setup**: A simple scene with reflective and emissive spheres.
- **Complex Objects**: Add custom planes and cubes using addPlane and addCube functions for detailed 3D models.