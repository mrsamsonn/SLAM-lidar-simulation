# SLAM LiDAR Mapping

This project is a hands-on implementation to explore and understand the concepts behind **Simultaneous Localization and Mapping (SLAM)**. It uses a simulated laser sensor to detect obstacles and create a map of the environment in real-time. The system continuously updates the map based on sensor data and accounts for uncertainty in the measurements, with added features for line segment extraction using seeded region growing.

Lidar Mapping Sim        |  Line Segment Extraction using Seed Region Growing Algorithm
:-------------------------:|:-------------------------:
<img src="https://github.com/user-attachments/assets/24646fbf-99d9-4112-bf62-31a8444018f5" alt="Lidar Mapping Sim" width="400"> |  <img src="https://github.com/user-attachments/assets/0c016ab9-066d-4b6f-aa84-449fa341d80a" alt="Mapping Feature Extraction" width="400">

## How It Works

The SLAM system works by simulating a laser sensor that scans the environment, detects obstacles, and generates a point cloud. The key features of this implementation are:

1. **Sensor Integration**: A simulated laser sensor scans the environment and returns data about obstacles. The laser sensor adds Gaussian noise to simulate uncertainty in real-world sensor measurements.
2. **Feature Detection**: The system detects line segments in the point cloud data using a seeded region growing algorithm, which is based on laser data. This allows the system to map not only individual obstacles but also linear structures (such as walls or straight edges).
3. **Data Collection**: The sensor gathers distance and angle measurements for obstacles detected in the sensor's range. This data is used for mapping and for segment extraction.
4. **Line Segment Extraction**: The system applies a seed-based region-growing method to extract line segments from the point cloud, improving the map by detecting continuous features in the environment.
5. **Map Update**: The map is updated dynamically in real-time by adding red dots at detected obstacles and visualizing extracted line segments in green.

### Key Components

| **Component**               | **Description**                                                                                              |
|-----------------------------|--------------------------------------------------------------------------------------------------------------|
| **Sensor Integration**       | A simulated laser sensor scans the environment and returns data about obstacles.                             |
| **Feature Detection**        | A seeded region-growing algorithm is applied to detect line segments in the point cloud.                     |
| **Data Collection**          | The laser sensor gathers distance and angle measurements from obstacles detected in its range.               |
| **Uncertainty Handling**     | Measurements are affected by uncertainty, modeled using a Gaussian distribution to simulate sensor noise.     |
| **Distance Calculation**     | The distance to obstacles is calculated using the Euclidean distance formula: `sqrt((x2 - x1)^2 + (y2 - y1)^2)`. |
| **Data Storage**             | Detected obstacle positions and line segments are stored in a point cloud, ensuring no duplicate points.      |
| **Map Update**               | The map is updated with red points for obstacles and green points for line segments based on sensor data.     |
| **Visualization**            | The map is visualized using Pygame, where obstacles are shown as red dots and line segments as green.         |

## New Features and Enhancements

1. **Line Segment Extraction with Seeded Region Growing**: 
    - The system now includes a seeded region growing algorithm for extracting line segments from LiDAR data. This allows for more complex features, like walls or edges, to be detected and mapped in the environment.

2. **Real-Time Map Updates**: 
    - The map is continuously updated with sensor data, where obstacles are visualized as red dots, and detected line segments are drawn in green.

3. **Dynamic Sensor Control**:
    - The system includes a feature where the laser sensor's data collection can be toggled based on mouse focus, providing real-time interaction during the simulation.

## Key Algorithms and Calculations

- **Uncertainty Model**: The distance and angle measurements are perturbed using a Gaussian distribution, which is implemented as `uncertainty_add()` in the code. This adds randomness to the measurements, simulating real-world sensor noise.

  ```python
  def uncertainty_add(distance, angle, sigma):
      mean = np.array([distance, angle])
      covariance = np.diag(sigma ** 2)
      distance, angle = np.random.multivariate_normal(mean, covariance)
      return [distance, angle]


## Research and Resource
[A line segment extraction algorithm
using laser data based on seeded
region growing - Haiming Gao , Xuebo Zhang, Yongchun Fang and Jing Yuan](https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245)

## Credits

This project was inspired by and credits the YouTube channel [Hobby Coding](https://youtube.com/@hobby_coding?si=fAOj5XnREPiPgcBT) for providing valuable insights and tutorials that helped in understanding the concepts behind this implementation.

