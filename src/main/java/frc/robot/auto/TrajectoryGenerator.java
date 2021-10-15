package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    TrajectoryConfig forwardConfigFast =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics, Constants.kMaxSpeedMetersPerSecond))
            .setReversed(false);

    TrajectoryConfig forwardConfigSlow =
        new TrajectoryConfig(
            Constants.kMinSpeedMetersPerSecond, 
            Constants.kMinAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics, Constants.kMaxSpeedMetersPerSecond))
            .setReversed(false);

    TrajectoryConfig reverseConfigFast =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics, Constants.kMaxSpeedMetersPerSecond))
            .setReversed(true);

    TrajectoryConfig reverseConfigSlow =
        new TrajectoryConfig(
                Constants.kMinSpeedMetersPerSecond, 
                Constants.kMinAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics, Constants.kMaxSpeedMetersPerSecond))
            .setReversed(true);

    public ProfiledPIDController thetaController =
        new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);

    // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    public Trajectory getDriveStraight() {
        return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(0))),
            forwardConfigSlow
            );
    }

    public Trajectory getDriveStraightReversed() {
        return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d()),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(Units.degreesToRadians(0))),
            reverseConfigSlow
            );
    }
}