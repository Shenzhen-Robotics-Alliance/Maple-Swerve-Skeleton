// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveTrainConstants;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean configurationFailed = false;

        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Rotation2d[] odometryYawPositions = new Rotation2d[DriveTrainConstants.ODOMETRY_CACHE_CAPACITY];
        public double yawVelocityRadPerSec = 0.0;
        public double pitchRad = 0.0;
        public double rollRad = 0.0;
    }

    void updateInputs(GyroIOInputs inputs);
}
