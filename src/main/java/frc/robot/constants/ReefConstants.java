package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.commands.reefscape.ReefAlignment;

public class ReefConstants {
    public static void loadStatic() {
        ReefAlignment.BranchTarget target1 = REEF_ALIGNMENT_POSITIONS_BLUE[0];
        ReefAlignment.BranchTarget target2 = REEF_ALIGNMENT_POSITIONS_RED[0];
    }

    public static final Distance ROBOT_TO_TARGET_DISTANCE = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
            // for simulation
            ? Centimeters.of(37.3)
            // for real robot (measure this on field)
            // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - X Axis
            : Centimeters.of(45.2);

    // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - Y Axis - Take Absolute Value
    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = new ReefAlignment.BranchTarget[] {
        // lower side
        ReefAlignment.BranchTarget.measured(
                18, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                18, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // lower right
        ReefAlignment.BranchTarget.measured(
                17, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                17, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper right
        ReefAlignment.BranchTarget.measured(
                22, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                22, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper side
        ReefAlignment.BranchTarget.measured(
                21, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                21, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper left
        ReefAlignment.BranchTarget.measured(
                20, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                20, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // lower left
        ReefAlignment.BranchTarget.measured(
                19, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                19, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // algae
        ReefAlignment.BranchTarget.measured(
                18, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                17, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                22, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                21, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                20, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                19, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = new ReefAlignment.BranchTarget[] {
        // lower side
        ReefAlignment.BranchTarget.measured(7, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                7, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // lower right
        ReefAlignment.BranchTarget.measured(8, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                8, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper right
        ReefAlignment.BranchTarget.measured(9, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                9, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper side
        ReefAlignment.BranchTarget.measured(
                10, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                10, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // upper left
        ReefAlignment.BranchTarget.measured(
                11, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                11, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // lower left
        ReefAlignment.BranchTarget.measured(6, ReefAlignment.Side.LEFT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(16.0)),
        ReefAlignment.BranchTarget.measured(
                6, ReefAlignment.Side.RIGHT, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(15.9)),
        // algae
        ReefAlignment.BranchTarget.measured(
                7, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                8, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                9, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                10, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                11, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
        ReefAlignment.BranchTarget.measured(
                6, ReefAlignment.Side.CENTER, ROBOT_TO_TARGET_DISTANCE, Centimeters.of(0.0)),
    };
}
