package frc.robot.utils;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class PPRobotConfigPrinter {
    public static void printConfig(RobotConfig ppRobotConfig) {
        System.out.println("PathPlanner Robot Config: ");
        System.out.println("\tRobot Mass (kg): " + ppRobotConfig.massKG);
        System.out.println("\tRobot MOI (kg * m^2): " + ppRobotConfig.MOI);
        System.out.println("\tRobot isHolonomic: " + ppRobotConfig.isHolonomic);

        // Print module locations
        System.out.println("\tModule Locations:");
        for (Translation2d location : ppRobotConfig.moduleLocations) {
            System.out.println("\t\t" + location);
        }

        // Print module config
        System.out.println("\tModule Config: ");
        printModuleConfig(ppRobotConfig.moduleConfig);
    }

    private static void printModuleConfig(ModuleConfig ppModuleConfig) {
        System.out.println("\t\tWheel Radius (m): " + ppModuleConfig.wheelRadiusMeters);
        System.out.println("\t\tMax Drive Velocity (m/s): " + ppModuleConfig.maxDriveVelocityMPS);
        System.out.println("\t\tWheel Coefficient of Friction: " + ppModuleConfig.wheelCOF);
        System.out.println("\t\tDrive Current Limit (A): " + ppModuleConfig.driveCurrentLimit);
        System.out.println("\t\tMax Drive Velocity (rad/s): " + ppModuleConfig.maxDriveVelocityRadPerSec);
        System.out.println("\t\tTorque Loss: " + ppModuleConfig.torqueLoss);

        // Print motor model
        printMotorModel(ppModuleConfig.driveMotor);
    }

    private static void printMotorModel(DCMotor dcMotor) {
        System.out.println("\t\tDCMotor Model:");
        System.out.println("\t\t\tNominal Voltage (V): " + dcMotor.nominalVoltageVolts);
        System.out.println("\t\t\tStall Torque (N·m): " + dcMotor.stallTorqueNewtonMeters);
        System.out.println("\t\t\tStall Current (A): " + dcMotor.stallCurrentAmps);
        System.out.println("\t\t\tFree Current (A): " + dcMotor.freeCurrentAmps);
        System.out.println("\t\t\tFree Speed (rad/s): " + dcMotor.freeSpeedRadPerSec);
        System.out.println("\t\t\tInternal Resistance (Ω): " + dcMotor.rOhms);
        System.out.println("\t\t\tVelocity Constant (rad/s/V): " + dcMotor.KvRadPerSecPerVolt);
        System.out.println("\t\t\tTorque Constant (N·m/A): " + dcMotor.KtNMPerAmp);
    }
}
