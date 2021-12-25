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

        Trajectory m_LastTrajectory;

        public Trajectory getLastTrajectory() {
                return m_LastTrajectory;
        }

        public ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController, 0, 0,
                        Constants.kThetaControllerConstraints);

        public ProfiledPIDController getThetaController() {
                return thetaController;
        }

        TrajectoryConfig forwardConfigFast = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics,
                                                        Constants.kMaxSpeedMetersPerSecond))
                                        .setReversed(false);

        TrajectoryConfig forwardConfigSlow = new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                        Constants.kMinAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics,
                                                        Constants.kMaxSpeedMetersPerSecond))
                                        .setReversed(false);

        TrajectoryConfig reverseConfigFast = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics,
                                                        Constants.kMaxSpeedMetersPerSecond))
                                        .setReversed(true);

        TrajectoryConfig reverseConfigSlow = new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                        Constants.kMinAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics)
                                        .addConstraint(new SwerveDriveKinematicsConstraint(Constants.kDriveKinematics,
                                                        Constants.kMaxSpeedMetersPerSecond))
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
}