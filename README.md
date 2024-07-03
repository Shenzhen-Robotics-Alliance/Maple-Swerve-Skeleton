# The 🍁 Swerve Skeleton

---

Iron Maple’s Swerve Drive Skeleton Project

High-Freq Odometry, Log-Replay, Vision Odometry, Auto-Alignment, Decisive Auto, and Digital Twin, all set to go!

> 🙏 We extend our deepest appreciation to [Littleton Robotics](https://www.littletonrobotics.org/) for their [open source projects](https://github.com/Mechanical-Advantage), which have made this project possible.
>
> ⚖️ This project is based on the [Advanced Swerve Drive Example](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main) from [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/), which is under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html#license-text) license. All teams are welcome to use this project, provided that you adhere to the [Advantage Kit License](./AdvantageKit-License.md).

## 🕹️Driving

- **250HZ Odometer**, Time-Synchronized with CAN FD
- **A Star Driving**
- **Skid Detection** inspired by [1690 online software session](https://youtu.be/N6ogT5DjGOk?feature=shared&t=1674)
- **Drive Wheels Feedforward** with a Look-Up-Table.
- **Acceleration Constraining** during TeleOp.
- **Robot Configuration Storage** in JSON files.
- **Robot Status Visualization** through [Elastic Dashboard](https://github.com/Gold872/elastic-dashboard), [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope), LED and [Network Alerts](https://github.com/Mechanical-Advantage/NetworkAlerts)

![Screenshot 2024-06-17 005557.png](./media/Screenshot_2024-06-17_005557.png)

## 🤖Auto

- **Decisive Autonomous Framework** with custom auto-follower, allowing robots to “react to its surroundings”.
- **Path-Planning**, [PathPlanner](https://github.com/mjansen4857/pathplanner) and [Choreo](https://github.com/SleipnirGroup/Choreo) both supported.

## 📝Logging

Built Upon [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit), all sensor inputs are logged and recorded for after-match analysis.

## 👁️Vision

[GitHub - Shenzhen-Robotics-Alliance/FRC-Phantom-Vision: a rapid, powerful, easy-to-use and open-source vision framework for FRC](https://github.com/Shenzhen-Robotics-Alliance/FRC-Phantom-Vision)

- **April-tag Robot Localization** Custom filtering mechanism and odometry calibration algorithm, also compatible with photon-vision.
- **Auto Alignment** command, set-to-go!
- **360° Game-Piece Detection** running on Jetson-Orin-Nano coprocessor.

## 👭Digital-Twin

A complete match simulation, with physics, score-count, human-players and even opponent robots.

- **Uses Actual Robot Code** in the simulator. This means that you can tune Auto-Stages, PIDs, Auto-Scoring functions and more.  The simulator display robots through https://github.com/Mechanical-Advantage/AdvantageScope.
- **Swerve-Drive Physics Simulation**, fine-tuned with experimental data measured in our training field, as well as real-life data gathered from real competitions this year.

    ![physics simulation 2.gif](./media/physics_simulation_2.gif)

- **2D Rigid-Body Collision Simulation** for game-pieces and robots on field.  Using open source 2d physics engine https://github.com/dyn4j/dyn4j.

    ![robot physics simulation.gif](./media/robot_physics_simulation.gif)

- **Opponent Robots Simulation** that can either be controlled by a gamepad to play defense or follow pre-stored cycle paths.

    ![Untitled video - Made with Clipchamp.gif](./media/Untitled_video_-_Made_with_Clipchamp.gif)
