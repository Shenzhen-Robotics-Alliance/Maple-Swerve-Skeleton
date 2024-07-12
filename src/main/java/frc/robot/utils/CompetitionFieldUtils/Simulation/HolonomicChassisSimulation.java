package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDrive;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

/**
 * simulates the behavior of holonomic chassis
 * the chassis will have a collision space
 * */
public abstract class HolonomicChassisSimulation extends Body {
    public final RobotProfile profile;
    public HolonomicChassisSimulation(RobotProfile profile) {
        this.profile = profile;

        super.addFixture(
                Geometry.createRectangle(profile.width, profile.height),
                profile.robotMass / (profile.height * profile.width),
                Constants.RobotPhysicsSimulationConfigs.ROBOT_BUMPER_COEFFICIENT_OF_FRICTION,
                Constants.RobotPhysicsSimulationConfigs.ROBOT_BUMPER_COEFFICIENT_OF_RESTITUTION
        );

        super.setMass(MassType.NORMAL);
        super.setLinearDamping(profile.linearVelocityDamping);
        super.setAngularDamping(profile.angularDamping);
    }

    public void setRobotPose(Pose2d robotPose) {
        super.transform.set(GeometryConvertor.toDyn4jTransform(robotPose));
    }

    public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
        super.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(chassisSpeeds));
        super.setAngularVelocity(chassisSpeeds.omegaRadiansPerSecond);
    }

    protected void simulateChassisBehaviorRobotRelative(ChassisSpeeds desiredChassisSpeedsRobotRelative) {
        simulateChassisBehavior(ChassisSpeeds.fromRobotRelativeSpeeds(desiredChassisSpeedsRobotRelative, getFacing()));
    }

    protected void simulateChassisBehavior(ChassisSpeeds desiredChassisSpeedsFieldRelative) {
        super.setAtRest(HolonomicDrive.isZero(desiredChassisSpeedsFieldRelative) && HolonomicDrive.isZero(getChassisSpeedsFieldRelative()));

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

    public Pose2d getPose() {
        return GeometryConvertor.toWpilibPose2d(getTransform());
    }

    public Rotation2d getFacing() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getChassisSpeedsFieldRelative() {
        return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity());
    }

    public ChassisSpeeds getChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsFieldRelative(), getFacing());
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
                    Constants.RobotPhysicsSimulationConfigs.FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ,
                    chassisGeneralInformation.getDoubleConfig("maxAngularVelocityRadiansPerSecond"),
                    Constants.RobotPhysicsSimulationConfigs.MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ,
                    Constants.RobotPhysicsSimulationConfigs.TIME_CHASSIS_STOPS_ROTATING_NO_POWER_SEC,
                    chassisGeneralInformation.getDoubleConfig("robotMassInSimulation"),
                    chassisGeneralInformation.getDoubleConfig("bumperWidthMeters"),
                    chassisGeneralInformation.getDoubleConfig("bumperLengthMeters")
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
