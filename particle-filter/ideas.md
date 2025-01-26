# Particle Filter for Robot Localization

## 1. **Initialization**

- **Particles Creation**: Initialize \( N \) particles randomly across the state space (typically \( (x, y, \theta) \) in 2D) or around a prior estimate of the robot's position.
  - Each particle \( p_i \) has a state \( (x_i, y_i, \theta_i) \).
  - Assign an initial weight \( w_i \) to each particle, usually \( w_i = 1/N \).

## 2. **Prediction Step (Motion Update)**

- Use the robot's odometry or motion model to predict the new state of each particle based on the control inputs (e.g., linear and angular velocities).
- Apply noise to the motion model to simulate real-world uncertainty. A common approach is to add Gaussian noise to the motion parameters.
- The motion model can be described as:
  \[
  \begin{cases}
  x' = x + (v \cdot \Delta t \cdot \cos(\theta + \omega \cdot \Delta t/2)) \\
  y' = y + (v \cdot \Delta t \cdot \sin(\theta + \omega \cdot \Delta t/2)) \\
  \theta' = \theta + \omega \cdot \Delta t
  \end{cases}
  \]
  where \( v \) is linear velocity, \( \omega \) is angular velocity, and \( \Delta t \) is the time step.

## 3. **Measurement Update (Weighting)**

- For each particle, use the lidar sensor readings to compute the "likelihood" of the observed measurements given the particle's predicted position.
- This involves comparing the lidar scan from the particle's perspective with the actual observed lidar scan and calculating a weight that represents how likely it is that the particle is at the correct position.
  - A common method is to use a probabilistic model like the Gaussian likelihood:
    \[
    w_i = \exp\left(-\frac{1}{2} \sum_j \frac{(z_j - z_i)^2}{\sigma^2}\right)
    \]
    where \( z_j \) is the measured distance by lidar and \( z_i \) is the predicted distance from the map, and \( \sigma \) is the measurement noise.

## 4. **Normalization of Weights**

- Normalize the weights of all particles such that:
  \[
  \sum\_{i=1}^N w_i = 1
  \]
- This step ensures the weights represent a valid probability distribution.

## 5. **Resampling**

- Resample particles according to their weights to create a new set of particles. This step reinforces particles with higher weights (more likely) and removes particles with lower weights (less likely).
- You can use **resampling algorithms** such as:
  - Systematic Resampling
  - Stochastic Universal Sampling
  - Residual Resampling

## 6. **Loop and Repeat**

- Continue looping through the prediction, measurement update, and resampling steps as the robot moves and collects new lidar data.

## **Additional Considerations**

- **Efficient Map Representation**: If using a grid map, consider using data structures like octrees for faster access.
- **Likelihood Function**: Fine-tune how you match lidar scans with the map for better accuracy.
- **Particle Count**: Balancing the number of particles is crucial for performance and accuracy.

This basic approach can be extended to 3D localization, include sophisticated motion models, adaptive resampling, etc., based on your specific robot and environment needs.
