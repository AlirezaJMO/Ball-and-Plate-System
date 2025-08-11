# Ball and Plate System

![Top View of the Ball and Plate System](TopView.gif)

## Introduction
The **Ball and Plate system** is an extension of the Ball and Beam setup, offering **four degrees of freedom** where a ball can freely roll on a flat surface.  
This system is **underactuated** and **nonlinear**, affected by friction and resistance to motion. It serves as a **benchmark** for testing various nonlinear control strategies.

The experimental setup includes:
- A flat plate
- Motors to tilt the plate along two perpendicular axes (X and Y)
- Actuators and a control system
- A camera to capture the ball's position

The plate tilts based on the system's control input, allowing for dynamic ball movement and providing a challenging environment to test different control methods.

## Project Objective
The **Ball and Plate (B&P)** system has direct applications in **robotic control of spherical objects** and is used as an **educational tool** in control engineering.

Unlike simpler models, the B&P system enables the study of:
- Stability control
- Trajectory/path tracking

It has been controlled and studied using various techniques, including:
- Linear control methods
- Optimal control
- Intelligent and nonlinear control strategies

## Methodology Summary

- **Dynamic Modeling**:  
  The dynamic equations of the system were derived using the **Euler-Lagrange method**.

- **Controller Design**:  
  Controller design and simulation were done in **MATLAB**.  
  Due to the nonlinear nature of the system, linearization was performed before designing a **state-feedback controller**.

- **Simulation Results**:  
  Simulations showed that the ball could be stabilized near the center of the plate, though small oscillations remained.

- **Implementation**:  
  A real prototype was built. A **camera** was used to capture the ball’s position and velocity for feedback.

- **Result Analysis**:  
  Real-world behavior closely matched simulation results, with the ball oscillating close to the center.

## Discussion and Conclusion
Simulation and implementation confirmed that **state-feedback control** could stabilize the ball near the center of the plate.  
However, due to the use of less precise motors and simplifications in the system model, some oscillations remained.

To further improve system performance:
- **Use more accurate motors**
- **Develop a more detailed and precise system model**

## Future Work
- Implement **higher precision motors** for better control of the plate.
- Explore and test **alternative control algorithms** such as:
  - PID control
  - Other advanced, intelligent, or nonlinear control strategies
