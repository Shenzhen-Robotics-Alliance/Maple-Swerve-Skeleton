# 🍁 Swerve Skeleton

---

6328 ([TBA](https://www.thebluealliance.com/team/6328/2024) | [Github](https://github.com/mechanical-advantage) | [Website](http://team6328.org/))'s [Advanced Swerve Drive Project](https://www.chiefdelphi.com/t/advantagekit-2024-log-replay-again/442968/54#advanced-swerve-drive-project-2) with enhanced physics simulation.

> 🙏 We extend our deepest appreciation to [Team 6328](https://www.littletonrobotics.org/) for their [open source project](https://github.com/Mechanical-Advantage), which have made this project possible.
>
> ⚖️ This project is based on an example from [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/), which is under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html#license-text) license. All teams are welcome to use it, provided that you adhere to the [Advantage Kit License](./AdvantageKit-License.md).

## Simulation Details

### The Field has Collision Space
The greatest distinction of the project is the implementation of [dyn4j](https://github.com/dyn4j/dyn4j), an open-source java physics engine, which enables us to add a collision detection for everything on the field, including obstacles, robots and game pieces.
![robot_physics_simulation.gif](media/robot_physics_simulation.gif)

### Swerve-Drive Physics Simulation
Fine-tuned with data measured in our training field, the swerve drive simulation provides an incredibly realistic drivetrain characteristics.
It simulates the behavior of each individual swerve module using [WPILib DCMotorSim](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/DCMotorSim.html) and use the data from 4 modules together to simulate the force on the drivetrain. 
[[Source Code](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/CompetitionFieldUtils/Simulations/SwerveDriveSimulation.java)]
![swervedrivesim.gif](media/swervedrivesim.gif)

### Game Pieces and Intake Simulation
In our simulation, game pieces on the field has collision spaces and can interact with the robot.
It also allows teams to simulate a fixed intake module on the robot that grabs a game pieces whenever in-contact.
[[Source Code](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/CompetitionFieldUtils/Simulations/IntakeSimulation.java) | [Example Code](https://github.com/Shenzhen-Robotics-Alliance/5516-2024-OffSeason/blob/main/src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java)]
![intakesim.gif](media/intakesim.gif)

### Opponent Robots Simulation
Simulated robots that can either be controlled by a gamepad to play defense or follow pre-stored cycle paths.
Just as real robots, opponent robots have collision spaces.
This allows drivers to practice defense/offense.
[[Source Code](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/CompetitionFieldUtils/Simulations/OpponentRobotSimulation.java)]
![opponentrobotsim.gif](media/opponentrobotsim.gif)

### Odometry Simulation

Odometry Simulation with realistic characteristics such as skidding and the IMU drifts.  
[[Source Code](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/subsystems/drive/IO/ModuleIOSim.java) | [Video](https://youtu.be/ersRWIzC0zc)]
![Odometry Sim](media/odometrysim.gif)
### Vision Simulation
Built Upon [PhotonLib Camera Sim](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html), our vision simulation behaves exactly the same as a real vision system
The system uses April Tags to position the robot on field, and teams can tune the system without a field
![visionsim1.gif](media/visionsim1.gif)
We also Provide a convenient feature that generates [Advantage Scope Fix Camera Config](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/CustomConfigs/PhotonCameraProperties.java) automatically, which allows you to see the camera's view in advantage scope with easy steps to set up, [Source Code](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/utils/CustomConfigs/PhotonCameraProperties.java)
![visionsim2.gif](media/visionsim2.gif)

## Setup Guide

- ### CTRE Chassis
  1. Generate [TunerConstants.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/constants/TunerConstants.java) Using [CTRE's Swerve Project Generator](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html).
  2. in your IDE, replace all `private` tags with `public`
  3. delete the last two lines 
    ``` Java
    // TODO: the last two lines MUST be removed
    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
      FrontRight, BackLeft, BackRight);
    ```
  4. Edit a few variables in [DriveTrainConstants.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/constants/DriveTrainConstants.java)
    ```java
    /**
    * numbers that needs to be changed to fit each robot
    * TODO: change these numbers to match your robot
    * */
    public static final double
      WHEEL_COEFFICIENT_OF_FRICTION = 0.95,
      ROBOT_MASS_KG = 40; // with bumpers
  
    /**
    * TODO: change motor type to match your robot 
    * */
    public static final DCMotor
      DRIVE_MOTOR = DCMotor.getKrakenX60(1),
      STEER_MOTOR = DCMotor.getFalcon500(1);
    ```
    5. Try Driving Your Chassis On Field!!!
- ### Rev Chassis
    > ⚠️ This project **supports** Rev hardware.  However, it has not been tested on a physical Rev Chassis simply because **we don't have one**.  If your team has an Rev Chassis and willing to test and debug our code, you are absolutely welcome and appreciated.
    
    1. in [RobotContainer.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/RobotContainer.java) line 100, change all `ModuleIOTalon` to `ModuleIOSpark`
       ``` java                 
       /* CTRE Chassis: */
       drive = new SwerveDrive(
         new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
         new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, "FrontLeft"),
         new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.FrontRight, "FrontRight"),
         new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackLeft, "BackLeft"),
         new ModuleIOTalon(TunerConstants.DrivetrainConstants, TunerConstants.BackRight, "BackRight")
       );

       /* REV Chassis */
       drive = new SwerveDrive(
       SwerveDrive.DriveType.CTRE_ON_CANIVORE,
         new GyroIOPigeon2(TunerConstants.DrivetrainConstants),
         new ModuleIOSpark(0),
         new ModuleIOSpark(1),
         new ModuleIOSpark(2),
         new ModuleIOSpark(3)
       );
       ```
    2. in [ModuleIOSpark.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/subsystems/drive/IO/ModuleIOSpark.java), configure the motor IDs as well as the encoder offsets
    3. In [DriveTrainConstants.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/constants/DriveTrainConstants.java), replace all references to `TunerConstants` with actual data
## Running Simulation
1. in [Robot.java](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton/blob/main/src/main/java/frc/robot/Robot.java) line 19, change `JAVA_SIM_MODE` to `SIM` to run physics simulation or change it to `REPLAY` to perform [6328](https://github.com/Mechanical-Advantage)'s [Log-Replay Technology](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/WHAT-IS-ADVANTAGEKIT.md).
    ```java
    // Run physics simulation
    private static final Constants.RobotMode JAVA_SIM_MODE = Constants.RobotMode.SIM;
    // Run Log-Replay technology
    private static final Constants.RobotMode JAVA_SIM_MODE = Constants.RobotMode.REPLAY;
    ```
  
    > 💡 There is **no need** to change the mode to `REAL` when deploying to real robot as the program will automatically switch to `REAL` mode if it is detected to be running on a real robot
    ```java
    public static final Constants.RobotMode CURRENT_ROBOT_MODE = isReal() ? Constants.RobotMode.REAL : JAVA_SIM_MODE;
    ```


2. Run the [WPILib Java Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html) and connect a gamepad to port 0 
    
3. Open [Advantage Scope](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/INDEX.md) and [connect to simulator](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html),
add a new [3D Field Widget](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md)

4. Drag `AdvantageKit/RealOutputs/Field/Robot` to `2D Poses`, make it `Robot` (this is where your robot **actually** is in the simulation world)
    
    Drag `AdvantageKit/RealOutputs/Field/Robots` to `2D Poses`, make it `Red Ghost` (these are opponent robots)
    
    Drag `AdvantageKit/RealOutputs/Odometry/Trajectory` to `2D Poses`, make it `Trajectory` (this is the current activate path)
    
    Drag `AdvantageKit/RealOutputs/Odometry/RobotPosition` to `2D Poses`, make it `Blue Ghost` (this is where the odometry think the robot is)

5. Open three [Sendable Choosers](https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html) in `SmartDashboard/FieldSimulation/` called `OpponentRobot1/2/3 Behavior` in the dashboard.  Select `Auto Cycle` to let AI control these robots to do cycles, or select `Joystick Control Left/Right-Handed` to control them manually with another gamepad (port 1/2/3 for opponent robot 1/2/3)

## Setting Up Vision
Drag `AdvantageKit/RealOutputs/Odometry/ValidPoseEstimations` to `3D Poses`, make it `Blue Ghost` (this is where the robot think it is)
