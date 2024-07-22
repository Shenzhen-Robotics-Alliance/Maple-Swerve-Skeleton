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

/**
 * simulates the physics behavior of holonomic chassis,
 * with respect to its collision space, friction and motor propelling forces
 * */
public abstract class HolonomicChassisSimulation extends Body implements RobotOnFieldDisplay {
    public final RobotProfile profile;
    public HolonomicChassisSimulation(RobotProfile profile) {
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
    }

    public void setSimulationWorldPose(Pose2d robotPose) {
        super.transform.set(GeometryConvertor.toDyn4jTransform(robotPose));
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

    public void runRawChassisSpeeds(ChassisSpeeds desiredChassisSpeedsRobotRelative) {
        simulateChassisBehavior(ChassisSpeeds.fromRobotRelativeSpeeds(desiredChassisSpeedsRobotRelative, getPose().getRotation()));
    }

    protected void simulateChassisBehavior(ChassisSpeeds desiredChassisSpeedsFieldRelative) {
        //        super.setAtRest(
//                HolonomicDrive.isZero(desiredChassisSpeedsFieldRelative)
//                        && HolonomicDrive.isZero(getMeasuredChassisSpeedsFieldRelative())
//        );
        super.setAtRest(false);

        final Vector2 desiredLinearMotionPercent = GeometryConvertor.toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative).multiply(1/ profile.robotMaxVelocity);
        simulateChassisTranslationalBehavior(desiredLinearMotionPercent);

        final double desiredRotationalMotionPercent = desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity;
        simulateChassisRotationalBehavior(desiredRotationalMotionPercent);
    }

    private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
        final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
        if (robotRequestedToMoveLinearly) {
            super.applyForce(new Force(
                    desiredLinearMotionPercent.multiply(this.profile.propellingForce)
            ));
            return;
        }

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

        final double actualRotationalMotionPercent = getAngularVelocity() / profile.maxAngularVelocity,
                frictionalTorqueMagnitude = this.profile.angularFrictionAcceleration * super.getMass().getInertia();
        if (actualRotationalMotionPercent > 0.03)
            super.applyTorque(Math.copySign(frictionalTorqueMagnitude, -super.getAngularVelocity()));
        else
            super.setAngularVelocity(0);
    }

    @Override
    public Pose2d getPose2d() {
        return getPose();
    }

    public Pose2d getPose() {
        return GeometryConvertor.toWpilibPose2d(getTransform());
    }

    public ChassisSpeeds getMeasuredChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getMeasuredChassisSpeedsFieldRelative(), getPose().getRotation());
    }

    public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity());
    }

    public double getChassisMaxLinearVelocity() {
        return profile.robotMaxVelocity;
    }

    public double getChassisMaxAngularVelocity() {
        return profile.maxAngularVelocity;
    }

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
    }
}