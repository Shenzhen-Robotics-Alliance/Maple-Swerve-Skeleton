package frc.robot.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberSim extends ElevatorSim {
    private final double ELEVATOR_MIN_HEIGHT_METERS, ELEVATOR_MAX_HEIGHT_METERS;
    private final ElevatorSim simWhenRobotClimbing;
    private boolean isRobotClimbing;
    /**
     * Assumption: robotMassKg > climbModuleMassKg
     * */
    public ClimberSim(DCMotor gearbox, double gearing, double climbModuleMassKg, double robotMassKg, double drumRadiusMeters, double minHeightMeters, double maxHeightMeters) {
        super(gearbox, gearing, climbModuleMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, true, minHeightMeters);
        this.simWhenRobotClimbing = new ElevatorSim(
                gearbox, gearing,
                robotMassKg - climbModuleMassKg,
                drumRadiusMeters,
                0, maxHeightMeters-minHeightMeters, // we simulate how much distance the robot have climbed (in comparison to maxHeight) downwards
                true,
                maxHeightMeters-minHeightMeters // the climber is at minHeightMeters in the beginning, so climber simulation is at the highest climbing position
        );
        this.ELEVATOR_MIN_HEIGHT_METERS = minHeightMeters;
        this.ELEVATOR_MAX_HEIGHT_METERS = maxHeightMeters;
    }

    public void startClimb() {
        isRobotClimbing = true;
    }

    public void stopClimb() {
        isRobotClimbing = false;
    }

    public void copyElevatorSimStateToClimberSim() {
        final double CLIMB_POSITION = ELEVATOR_MAX_HEIGHT_METERS - super.getPositionMeters(),
                CLIMB_VELOCITY = -super.getVelocityMetersPerSecond();
        simWhenRobotClimbing.setState(CLIMB_POSITION, CLIMB_VELOCITY);
    }

    public void copyClimberSimStateToElevatorSim() {
        final double ELEVATOR_POSITION = ELEVATOR_MAX_HEIGHT_METERS - simWhenRobotClimbing.getPositionMeters(),
                ELEVATOR_VELOCITY = -simWhenRobotClimbing.getVelocityMetersPerSecond();
        super.setState(ELEVATOR_POSITION, ELEVATOR_VELOCITY);
    }

    @Override
    public void setInputVoltage(double volts) {
        simWhenRobotClimbing.setInputVoltage(-volts);
        super.setInputVoltage(volts);
    }

    @Override
    public void update(double dtSeconds) {
        if (isRobotClimbing) {
            simWhenRobotClimbing.update(dtSeconds);
            copyClimberSimStateToElevatorSim();
            super.update(1e-4); // flush results
        } else {
            super.update(dtSeconds);
            copyElevatorSimStateToClimberSim();
            simWhenRobotClimbing.update(1e-4); // flush results
        }
    }
}
