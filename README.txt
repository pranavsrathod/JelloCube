CSCI 520, Assignment 1

Pranav Rathod

================

Accomplishments:
For this assignment, I successfully implemented a physically-based simulation of a Jello Cube using a mass-spring system. 
The cube deforms, oscillates, and reacts realistically to forces based on Newton's laws of motion, Hooke's law, and damping forces.

- Implemented computeAcceleration() in physics.cpp to correctly calculate acceleration based on forces from:
  - Structural, shear, and bend springs
  - External force fields (if applicable)
  - Collision response with the bounding box

  The elasticity of the jello cube is modeled using a mass-spring system, where forces between connected points follow Hooke's Law:

      F = -k * (x - x0)

  where:
  - F is the restoring force,
  - k is the spring constant (elasticity),
  - x is the current distance between two points,
  - x0 is the rest length of the spring.

  Each mass point is connected to others via structural, shear, and bend springs, ensuring the cube maintains its shape while allowing realistic deformations. 
  The simulation computes forces for all these spring types efficiently using structured loops.

Damping Forces:
Damping is applied to prevent excessive oscillations and ensure a physically stable simulation. It follows the damping force equation:

      F_d = -k_d * ((v_A - v_B) . L) / |L|

  where:
  - k_d is the damping coefficient,
  - v_A - v_B is the velocity difference between connected mass points,
  - L is the displacement vector between them.

  Damping is applied to all types of springs to realistically absorb energy from the system.

- Integrated Euler and RK4 methods to solve the equations of motion and update the cube's state over time.

- Implemented collision detection and response for interactions with the bounding box using the penalty method.

  Collision detection ensures the cube remains within a bounding box [-2,2]^3. If a mass point moves outside this range, 
  a penalty force is applied using an artificial spring, pushing it back into the box:

      F_c = k_c * (p_boundary - p)

  where:
  - k_c is the collision elasticity constant,
  - p_boundary is the closest valid position inside the box,
  - p is the current position of the point.

  A damping force is also applied during collisions to reduce sudden rebounds.

External Forces (Force Field):
A spatially varying external force field influences the cube if defined in the .w world file. The force at each point is obtained using trilinear interpolation, ensuring smooth variations. 
Forces are computed using:

1. Grid Indexing - The mass point’s position is mapped to the discrete force field grid.
2. Interpolation - Surrounding grid points are weighted to obtain a continuous force.

- Ensured proper OpenGL rendering, allowing the user to:
  - Toggle between wireframe and solid shading modes
  - View the cube from different angles using mouse controls
  - Toggle display of structural, shear, and bend springs for debugging

================

Extra Credit Implemented:
- Implemented collision detection and response with an **inclined plane**.

Inclined Plane Collision:
- The inclined plane is represented by the equation:  
      
      a * x + b * y + c * z + d = 0

  where `a, b, c, d` are read from the world file if an inclined plane is present.

- Rendering:
  - The inclined plane is **dynamically drawn** in `showInclinedPlane()` in `jello.cpp` using four computed boundary points.

- Collision Detection:
  - Each point's position is checked against the plane equation.
  - If the point is **below the plane** (distance < 0), a **penalty force** is applied to push it back.

- Collision Response:
  - A force is computed based on **penetration depth** and the plane’s normal.
  - An additional **damping force** is applied opposite to the velocity to prevent excessive bouncing.
  - This is implemented in `applyCollisionForces()` in `physics.cpp`.

The addition of the inclined plane allows for more complex interactions and enhances the realism of the jello simulation.

================================================================================

Development Environment:
- OS: MacOS
- Editor: Visual Studio Code
- Compiler: Clang++ (supports C++17)
- Build System: Makefile (provided with the starter code)
- Libraries Used: OpenGL, GLUT (Mac Frameworks)

================

Compilation Instructions:
This project was compiled using the provided Makefile. To build and run:

1. Compile the project: make
2. Run the simulation with a world file: ./jello world/jello.w
3. To create a world file: ./createWorld
4. To clean compiled files: make clean

Note: The Makefile provided automatically detects MacOS and links the appropriate OpenGL/GLUT frameworks.

================

Notes:
- The project has been thoroughly tested using the provided world files.
- The simulation runs efficiently at interactive frame rates (>15fps at 640x480 resolution).
- The submission includes the Mac executable. Since grading is done on Windows, please use the provided source code and recompile on a Windows system if needed.