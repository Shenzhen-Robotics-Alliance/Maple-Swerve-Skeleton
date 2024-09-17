# Demo Project: 5516 Crescendo Off-Season Robot "11:59"

## What is this?

This is a demo project for [Iron Maple](https://github.com/Shenzhen-Robotics-Alliance)'s Maple Swerve Skeleton,
a swerve drive framework designed with an emphasis on realistic robot simulation.

https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton

You can, of course, play this demo like a game, but keep in mind that the graphics and playability are not on the same level as driving games built on dedicated game engines, such as [MoSimulator](https://mostudios.itch.io/mosimulator).

This project prioritizes realism. Everything that happens to your robot in real life also happens in the simulator:
The mechanisms are simulated by using [WPILib Physics Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html);
the auto-aim feature in the simulator is powered by a vision system, which is simulated using [PhotonLib Simulation](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html);
the simulator models both the motors' propelling forces and the wheels' gripping forces to deliver a highly realistic drivetrain simulation.

Additionally, if you use Maple Swerve Skeleton to create your own robot code, the key bindings and program logic in the simulator will match the real robot perfectly. This is because the exact same code that runs on the real robot is also running within the [Java Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html).

The realistic simulation is ideal for driver practice, allowing you to refine your skills in a true-to-life environment. Additionally, AI robots can assist in practicing various strategies, such as feed-and-shoot drills, providing a versatile training experience.

## Other Capabilities
In addition to driver practice, the realistic simulation is a valuable tool for programmers. It allows you to fine-tune your code or adjust autonomous stages without needing the physical robot, making development and testing more efficient.

During the build season, this means you can begin writing robot code as soon as the robot's design is finalized. By the time the robot is fully built, you'll already have a well-tested robot code in the simulator, and you might even have a conceptual autonomous stage ready to go.

## Playing this demo
> ⚠️ If you are not used to FRC Java programming, you might need some help from a programmer in your team
> 
1. Open the latest build of [Advantage-Scope](https://github.com/Mechanical-Advantage/AdvantageScope), select `Help -> Use Custom Assets Folder`, select `Maple-Swerve-Skelton/example/5516-2024-OffSeason/AdvantageScopeAssets`
2. In [WPILib Vscode](), run `Simulate Java` according to [this guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html#introduction-to-robot-simulation)
3. In Advantage Scope:
    - Drag `AdvantageKit/RealOutputs/Field/Robot` to `2D Poses`, make it `Robot`
    - Drag `AdvantageKit/RealOutputs/Field/Note` to `3D Poses`, make it `Note`
    - Drag `AdvantageKit/RealOutputs/Shooter/MechanismPose` to `3D Poses`, make it `Component (Robot)`
    - Drag `AdvantageKit/RealOutputs/Field/RobotBlue` to `3D Poses`, make it `Blue Ghost` (these are blue AI-robots)
    - Drag `AdvantageKit/RealOutputs/Field/RobotRed` to `3D Poses`, make it `Red Ghost` (these are red AI-robots)
4. Double-Right-Click the 3D field, Select `DriverStation -> ` and choose a driver-station view
5. Attach an Xbox Game Controller to [Simulation GUI](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html)
6. in the Simulation GUI, select `Teleoperated` to begin Teleop simulation
7. You can also select autonomous mode in `SmartDashboard/SelectAuto` in your dashboard, select `Autonomous` in the simulation GUI.  If you want the robot to also shoot preload, grab a note in `Teleoperated` mode, select `Disable` mode and then select `Autonomous` mode.

## Creating Simulation for Your Own
With just a few hours of setup, you can adapt **Maple Swerve Skeleton** for your own robot. Simply follow the steps outlined in the [Setup Guide](https://github.com/Shenzhen-Robotics-Alliance/Maple-Swerve-Skeleton?tab=readme-ov-file#setup-guide).
