package frc.robot.auto;

import static frc.robot.Constants.kDriveKinematics;
import static frc.robot.Constants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.kMinAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.kMinSpeedMetersPerSecond;
import static frc.robot.Constants.kPThetaController;
import static frc.robot.Constants.kThetaControllerConstraints;
import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.util.PathPlanner;

public class TrajectoryGenerator {
        private static final TrajectoryGenerator instance = new TrajectoryGenerator();

        public static TrajectoryGenerator getInstance() {
                return instance;
        }

        public TrajectoryGenerator() {
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

        Trajectory m_LastTrajectory;

        public Trajectory getLastTrajectory() {
                return m_LastTrajectory;
        }

        public ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0,
                        kThetaControllerConstraints);

        public ProfiledPIDController getThetaController() {
                return thetaController;
        }

        TrajectoryConfig forwardConfigFast = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared).setKinematics(kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                                                        kMaxSpeedMetersPerSecond))
                                        .setReversed(false);

        TrajectoryConfig forwardConfigSlow = new TrajectoryConfig(kMinSpeedMetersPerSecond,
                        kMinAccelerationMetersPerSecondSquared).setKinematics(kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                                                        kMaxSpeedMetersPerSecond))
                                        .setReversed(false);

        TrajectoryConfig reverseConfigFast = new TrajectoryConfig(kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared).setKinematics(kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                                                        kMaxSpeedMetersPerSecond))
                                        .setReversed(true);

        TrajectoryConfig reverseConfigSlow = new TrajectoryConfig(kMinSpeedMetersPerSecond,
                        kMinAccelerationMetersPerSecondSquared).setKinematics(kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                                                        kMaxSpeedMetersPerSecond))
                                        .setReversed(true);

        public Trajectory getDriveTest() {
                return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                        List.of(
                                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new Pose2d(0, 5, Rotation2d.fromDegrees(180))
                        ),
                        forwardConfigFast
                );
        }

        public Trajectory getPlannerTest() {
                return PathPlanner.loadPath("New Path", MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        }
}