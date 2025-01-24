# Algorithm Explain:
## Feedforward Angular Speed for Face-To-Target Control

To ensure the robot faces the target while moving, a closed-loop control system is required to continuously adjust the robot's heading towards the target. However, because the robot is constantly in motion, there will always be a delay in the response of the PID controller. This delay can cause the robot to lag behind the desired heading. To compensate for this, a feed-forward angular velocity is introduced, which proactively adjusts the robot's rotation based on its speed and direction, helping to reduce the delay and improve the robot's ability to maintain its alignment with the target in real-time.

In brief, given the target's position, the robot's position, and its velocity, we calculate the rate of change of the robot's desired rotation to maintain alignment with the target.

![alt text](<face to target.png>)

#### 1. **Calculate Target's Velocity Relative to the Robot**

First, the target's velocity relative to the robot is calculated in the field-origin frame. This is based on the measured chassis velocities:

\[
\mathbf{v}_{\text{target relative}} = (-v_{x}, -v_{y})
\]

Where:
- \( v_x \) and \( v_y \) are the field-relative velocities of the robot in the x and y directions.

#### 2. **Calculate Target's Desired Rotation**

The rotation needed to face the target is computed based on the target's position relative to the robot. If `shooterOptimization` is not provided, the robot should directly face the target’s position:

\[
\theta_{\text{target}} = \text{atan2}(y_{\text{target}} - y_{\text{robot}}, x_{\text{target}} - x_{\text{robot}})
\]

Alternatively, if `shooterOptimization` is provided, the desired facing direction is determined by the optimization algorithm:

\[
\theta_{\text{target}} = \text{ShooterFacing}(x_{\text{target}}, y_{\text{target}}, x_{\text{robot}}, y_{\text{robot}}, \mathbf{v}_{\text{robot}})
\]

Where \( \theta_{\text{target}} \) represents the desired rotational angle to face the target.


#### 3. **Calculate Target's Moving Direction**

The target’s moving direction is calculated from the target’s relative velocity:

\[
\theta_{\text{moving}} = \text{Angle of}(\mathbf{v}_{\text{target relative}})
\]

#### 4. **Calculate the Perpendicular Tangent Direction**

The perpendicular direction to the target's movement is computed by rotating the target's facing direction by \(90^\circ\):

\[
\theta_{\text{perpendicular}} = \theta_{\text{target}} + 90^\circ
\]

#### 5. **Calculate Tangential Velocity**

The tangential velocity is calculated based on the cosine of the angle between the target's moving direction and the perpendicular tangent direction:

\[
v_{\text{tangent}} = \|\mathbf{v}_{\text{target relative}}\| \cdot \cos(\theta_{\text{moving}} - \theta_{\text{perpendicular}})
\]

Where:
- \( \|\mathbf{v}_{\text{target relative}}\| \) is the magnitude (norm) of the target's velocity vector.

#### 6. **Calculate the Distance to the Target**

The distance between the robot and the target is computed:

\[
d_{\text{target}} = \|\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{robot}}\|
\]

Where \( \mathbf{p}_{\text{target}} \) and \( \mathbf{p}_{\text{robot}} \) are the positions of the target and the robot, respectively.

#### 7. **Feedforward Angular Velocity**

The feedforward angular velocity required to align with the target is calculated by dividing the tangential velocity by the distance to the target:

\[
\omega_{\text{feedforward}} = \frac{v_{\text{tangent}}}{d_{\text{target}}}
\]
