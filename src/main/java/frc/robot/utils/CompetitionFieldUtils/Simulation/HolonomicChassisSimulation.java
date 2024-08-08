package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RobotOnFieldDisplay;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

import static frc.robot.utils.MapleMaths.MapleCommonMath.constrainMagnitude;

/**
 * simulates the physics behavior of holonomic chassis,
 * with respect to its collision space, friction and motor propelling forces
 * */
public abstract class HolonomicChassisSimulation extends Body implements RobotOnFieldDisplay {
    public final RobotProfile profile;
    public HolonomicChassisSimulation(RobotProfile profile, Pose2d startingPose) {
        this.profile = profile;

        /* the bumper needs to be rotated 90 degrees */
        final double WIDTH_IN_WORLD_REFERENCE = profile.height,
                HEIGHT_IN_WORLD_REFERENCE = profile.width;
        super.addFixture(
                Geometry.createRectangle(WIDTH_IN_WORLD_REFERENCE, HEIGHT_IN_WORLD_REFERENCE),
                profile.robotMass / (profile.height * profile.width),
                Constants.RobotPhysicsSimulationConfigs.ROBOT_BUMPER_COEFFICIENT_OF_FRICTION,
                Constants.RobotPhysicsSimulationConfigs.ROBOT_BUMPER_COEFFICIENT_OF_RESTITUTION
        );

        super.setMass(MassType.NORMAL);
        super.setLinearDamping(profile.linearVelocityDamping);
        super.setAngularDamping(profile.angularDamping);
        setSimulationWorldPose(startingPose);
    }

    public void setSimulationWorldPose(Pose2d robotPose) {
        super.transform.set(GeometryConvertor.toDyn4jTransform(robotPose));
        super.linearVelocity.set(0, 0);
    }

    /**
     * sets the robot's speeds to a given chassis speeds
     * the robot's speeds will jump to the given speeds in a tick
     * this is different from runRawChassisSpeeds(), which applies forces on the chassis and accelerates smoothly according to physics
     * */
    protected void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        super.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(givenSpeeds));
        super.setAngularVelocity(givenSpeeds.omegaRadiansPerSecond);
    }

    public void simulateChassisBehaviorWithRobotRelativeSpeeds(ChassisSpeeds desiredChassisSpeedsRobotRelative) {
        simulateChassisBehaviorWithFieldRelativeSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(desiredChassisSpeedsRobotRelative, getObjectOnFieldPose2d().getRotation()));
    }

    protected void simulateChassisBehaviorWithFieldRelativeSpeeds(ChassisSpeeds desiredChassisSpeedsFieldRelative) {
        super.setAtRest(false);

        final Vector2 desiredLinearMotionPercent = GeometryConvertor
                .toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
                .multiply(1.0/ profile.robotMaxVelocity);
        simulateChassisTranslationalBehavior(Vector2.create(
                constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1),
                desiredLinearMotionPercent.getDirection()
        ));

        final double desiredRotationalMotionPercent = desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity;
        simulateChassisRotationalBehavior(constrainMagnitude(desiredRotationalMotionPercent, 1));
    }

    protected void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
        final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
        if (!robotRequestedToMoveLinearly) {
            simulateTranslationalFriction();
            return;
        }
        final Vector2 forceVec = desiredLinearMotionPercent.copy().multiply(this.profile.propellingForce);
        super.applyForce(new Force(forceVec));
    }

    protected void simulateTranslationalFriction() {
        final double actualLinearPercent = getLinearVelocity().getMagnitude() / profile.robotMaxVelocity;
        final boolean robotActuallyMovingLinearly = actualLinearPercent > 0.03;
        if (robotActuallyMovingLinearly)
            super.applyForce(new Force(
                    super.linearVelocity.getNormalized().multiply(-profile.frictionForce)
            ));
        else
            super.setLinearVelocity(new Vector2());
    }

    private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
        final double maximumTorque = this.profile.maxAngularAcceleration * super.getMass().getInertia();
        if (Math.abs(desiredRotationalMotionPercent) > 0.03) {
            super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
            return;
        }

        final double actualRotationalMotionPercent = Math.abs(getAngularVelocity() / profile.maxAngularVelocity),
                frictionalTorqueMagnitude = this.profile.angularFrictionAcceleration * super.getMass().getInertia();
        if (actualRotationalMotionPercent > 0.03)
            super.applyTorque(Math.copySign(frictionalTorqueMagnitude, -super.getAngularVelocity()));
        else
            super.setAngularVelocity(0);
    }

    @Override
    public Pose2d getObjectOnFieldPose2d() {
        return GeometryConvertor.toWpilibPose2d(getTransform());
    }

    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getMeasuredChassisSpeedsFieldRelative(), getObjectOnFieldPose2d().getRotation());
    }

    public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity());
    }

    /**
     * called in every iteration of sub-period
     * */
    public abstract void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds);

    public static final class RobotProfile {
        public final double
                robotMaxVelocity,
                robotMaxAcceleration,
                robotMass,
                propellingForce,
                frictionForce,
                linearVelocityDamping,
                maxAngularVelocity,
                maxAngularAcceleration,
                angularDamping,
                angularFrictionAcceleration,
                width,
                height;

        public RobotProfile(MapleConfigFile.ConfigBlock chassisGeneralInformation) {
            this(
                    chassisGeneralInformation.getDoubleConfig("maxVelocityMetersPerSecond"),
                    chassisGeneralInformation.getDoubleConfig("maxAccelerationMetersPerSecondSquared"),
                    chassisGeneralInformation.getDoubleConfig("maxAngularVelocityRadiansPerSecond"),
                    chassisGeneralInformation.getDoubleConfig("robotMassInSimulation"),
                    chassisGeneralInformation.getDoubleConfig("bumperWidthMeters"),
                    chassisGeneralInformation.getDoubleConfig("bumperLengthMeters")
            );
        }

        public RobotProfile(double robotMaxVelocity, double robotMaxAcceleration, double maxAngularVelocity, double robotMass, double width, double height) {
            this(
                    robotMaxVelocity,
                    robotMaxAcceleration,
                    Constants.RobotPhysicsSimulationConfigs.FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ,
                    maxAngularVelocity,
                    Constants.RobotPhysicsSimulationConfigs.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ,
                    Constants.RobotPhysicsSimulationConfigs.TIME_CHASSIS_STOPS_ROTATING_NO_POWER_SEC,
                    robotMass,
                    width, height
            );
        }

        public RobotProfile(double robotMaxVelocity, double robotMaxAcceleration, double floorFrictionAcceleration, double maxAngularVelocity, double maxAngularAcceleration, double timeChassisStopsRotating, double robotMass, double width, double height) {
            this.robotMaxVelocity = robotMaxVelocity;
            this.robotMaxAcceleration = robotMaxAcceleration;
            this.robotMass = robotMass;
            this.propellingForce = robotMaxAcceleration * robotMass;
            this.frictionForce = floorFrictionAcceleration * robotMass;
            this.linearVelocityDamping = robotMaxAcceleration / robotMaxVelocity;
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.angularDamping = maxAngularAcceleration / maxAngularVelocity;
            this.angularFrictionAcceleration = maxAngularVelocity / timeChassisStopsRotating;
            this.width = width;
            this.height = height;
        }

        @Override
        public String toString() {
            return String.format("RobotProfile { robotMaxVelocity=%.2f, robotMaxAcceleration=%.2f, robotMass=%.2f, " +
                            "propellingForce=%.2f, frictionForce=%.2f, linearVelocityDamping=%.2f, maxAngularVelocity=%.2f, " +
                            "maxAngularAcceleration=%.2f, angularDamping=%.2f, angularFrictionAcceleration=%.2f, width=%.2f, " +
                            "height=%.2f }",
                    robotMaxVelocity, robotMaxAcceleration, robotMass, propellingForce, frictionForce, linearVelocityDamping,
                    maxAngularVelocity, maxAngularAcceleration, angularDamping, angularFrictionAcceleration, width, height);
        }
    }
}
