ADCS (Attitude Determination and Control Subsystem) is the spacecraft subsystem responsible for determining and controlling the orientation, or attitude, of the vehicle in space. It ensures that the spacecraft is pointing in the right direction at the right time.

To achieve this, the subsystem combines different elements. Sensors such as sun sensors, star trackers, gyroscopes, or magnetometers, which provide information about the spacecraft’s current orientation. Estimation algorithms (e.g. Kalman Filter) process these measurements to determine the attitude with sufficient accuracy. Based on this information, control laws command actuators, such as reaction wheels, magnetorquers, or thrusters, in order to generate the torques needed to reach or maintain the desired orientation.

A properly functioning ADCS is essential for many mission operations, including pointing antennas toward Earth for communication, orienting solar panels toward the Sun for power generation, stabilizing the spacecraft after deployment (detumbling), or directing scientific instruments toward specific targets. Because of this, ADCS sits at the intersection of dynamics, control theory, sensor modeling, estimation and spacecraft operations, making it one of the most mathematically and physically rich subsystems of a spacecraft.

+---------------------------+
|   Mission Requirements    |
| (pointing, stability,     |
|  maneuvering constraints) |
+-------------+-------------+
              |
              v
+---------------------------+
|   Guidance / Target       |
|   Desired Attitude        |
| (reference quaternion or  |
|  pointing direction)      |
+-------------+-------------+
              |
              v
+----------------------+   +---------------------------+   +----------------------+
|       Sensors        |-->|   Attitude Determination  |-->|      Controller      |
|                      |   |   (Estimation Algorithms) |   |     (Control Law)    |
| - Sun sensors        |   |                           |   |                      |
| - Star trackers      |   | - Quaternion estimation   |   | - PD / PID control   |
| - Gyroscopes         |   | - Sensor fusion           |   | - B-dot detumbling   |
| - Magnetometers      |   | - Kalman filtering (EKF)  |   | - Momentum management|
| - Horizon sensors    |   |                           |   | - LQR / advanced     |
+----------+-----------+   +-------------+-------------+   +----------+-----------+
           |                             |                            |
           |                             |                            v
           |                             |               +--------------------------+
           |                             +-------------->|        Actuators         |
           |                                             |                          |
           |                                             | - Reaction wheels        |
           |                                             | - Magnetorquers (MTQ)    |
           |                                             | - Control moment gyros   |
           |                                             | - Thrusters              |
           |                                             | - Magnetic torquers      |
           |                                             +------------+-------------+
           |                                                          |
           |                                                          v
           |                                               +----------------------+
           +---------------------------------------------->|     Spacecraft      |
                                                           |      Attitude       |
                                                           |  (Rigid body dyn.)  |
                                                           +----------+-----------+
                                                                      |
                                                                      v
                                                              Feedback to sensors
