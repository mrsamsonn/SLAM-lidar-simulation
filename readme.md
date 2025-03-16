# SLAM LiDAR Mapping - Learning Project

This project is a hands-on implementation to explore and understand the concepts behind **Simultaneous Localization and Mapping (SLAM)**. It uses a simulated laser sensor to detect obstacles and create a map of the environment in real-time. The system continuously updates the map based on sensor data and accounts for uncertainty in the measurements.

![slam-demo](https://github.com/user-attachments/assets/24646fbf-99d9-4112-bf62-31a8444018f5)

## How It Works

The SLAM system works by simulating a laser sensor that scans the environment, detects obstacles, and generates a point cloud. Below is an overview of the key components and steps involved:

| **Component**              | **Description**                                                                                           |
|----------------------------|-----------------------------------------------------------------------------------------------------------|
| **Sensor Integration**      | A simulated laser sensor scans the environment and returns data about obstacles.                          |
| **Data Collection**         | The sensor gathers distance and angle measurements for obstacles detected in the sensor’s range.          |
| **Uncertainty Handling**    | The measurements are affected by uncertainty, modeled using a Gaussian distribution, to simulate sensor noise. |
| **Distance Calculation**    | The distance to obstacles is calculated using the Euclidean distance formula: `sqrt((x2 - x1)^2 + (y2 - y1)^2)`. |
| **Data Storage**            | The detected obstacle positions are stored in a point cloud, ensuring no duplicate points are added.       |
| **Map Update**              | The map is updated by adding red points at the obstacle positions based on sensor data.                   |
| **Visualization**           | The map is visualized using Pygame, where obstacles are shown as red dots on the environment map.          |

## Key Algorithms and Calculations

- **Uncertainty Model**: The distance and angle measurements are perturbed using a Gaussian distribution, which is implemented as `uncertainty_add()` in the code. This adds randomness to the measurements, simulating real-world sensor noise.
  
  ```python
  def uncertainty_add(distance, angle, sigma):
      mean = np.array([distance, angle])
      covariance = np.diag(sigma ** 2)
      distance, angle = np.random.multivariate_normal(mean, covariance)
      return [distance, angle]

## Credits

This project was inspired by and credits the YouTube channel [Hobby Coding](https://youtube.com/@hobby_coding?si=fAOj5XnREPiPgcBT) for providing valuable insights and tutorials that helped in understanding the concepts behind this implementation.

