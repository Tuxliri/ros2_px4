# Readme

## IMU Noise Parameter Conversion: Gazebo to OpenVINS/Kalibr

This document explains how the IMU noise parameters were converted from Gazebo sensor definitions to OpenVINS/Kalibr format.

### Gazebo IMU Sensor Parameters (IIM42653 Model)

#### Gyroscope Noise
- **Gazebo stddev**: 0.0008726646 rad/s (all axes)
- **Physical meaning**: 0.05 deg/s RMS noise

#### Accelerometer Noise  
- **Gazebo stddev**: 
  - X & Y axes: 0.00637 m/s²
  - Z axis: 0.00686 m/s²
- **Physical meaning**: 0.65 mg-rms (X&Y), 0.70 mg-rms (Z)

### Conversion Formulas

#### 1. Noise Density (White Noise)

**Formula**: `noise_density = gazebo_stddev`

**Explanation**: 
- Gazebo's `stddev` parameter represents the standard deviation of Gaussian white noise
- OpenVINS/Kalibr's `noise_density` is the continuous-time noise spectral density
- For discrete sensors, these are equivalent when properly normalized

**Applied Values**:
- **Gyroscope**: `gyroscope_noise_density = 0.0008726646` rad/s/√Hz
- **Accelerometer**: `accelerometer_noise_density = 0.0066` m/s²/√Hz
  - Used average of X,Y,Z: (0.00637 + 0.00637 + 0.00686) / 3 ≈ 0.0066

#### 2. Random Walk (Bias Drift)

**Formula**: `random_walk ≈ noise_density / time_constant`

**Explanation**:
- Random walk models the slow drift of sensor bias over time
- Not directly specified in Gazebo, so estimated based on typical MEMS IMU characteristics
- Conservative estimates for simulation environment

**Applied Values**:
- **Gyroscope**: `gyroscope_random_walk = 0.00001745329` rad/s²/√Hz
  - ≈ noise_density / 50 (typical time constant for bias stability)
- **Accelerometer**: `accelerometer_random_walk = 0.0001` m/s³/√Hz  
  - ≈ noise_density / 66 (conservative estimate)

#### 3. Update Rate

**Direct Match**: `update_rate = 250` Hz (matches Gazebo sensor update rate)

### Physical Interpretation

#### Gyroscope (0.05 deg/s RMS)
```
0.05 deg/s × (π/180) = 0.0008726646 rad/s
```

#### Accelerometer (0.65/0.70 mg-RMS)
```
X,Y: 0.65 mg × 9.80665 m/s²/g / 1000 = 0.00637 m/s²
Z:   0.70 mg × 9.80665 m/s²/g / 1000 = 0.00686 m/s²
```

### Notes

1. **White Noise**: Direct conversion from Gazebo stddev to noise density
2. **Bias Random Walk**: Estimated based on typical MEMS IMU characteristics
3. **Update Rate**: Matched to Gazebo sensor rate (250 Hz)
4. **Conservative Approach**: Random walk values are conservative for simulation stability

### References

- IIM42653 IMU Datasheet specifications
- OpenVINS noise model documentation
- Kalibr IMU calibration format
- Allan Variance analysis methods for IMU characterization
