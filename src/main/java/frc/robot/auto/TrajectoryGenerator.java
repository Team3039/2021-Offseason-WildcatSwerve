package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.util.PathPlanner;

import java.util.List;

import static frc.robot.Constants.DrivetrainCoefficients.*;
import static frc.robot.Constants.MotionConstraints.*;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();
    public ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0,
            kThetaControllerConstraints);
    Trajectory m_LastTrajectory;

    static class TrajectoryConfigs {
            static TrajectoryConfig forwardConfigFast = new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND,
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(kDriveKinematics)
                    .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                            MAX_VELOCITY_METERS_PER_SECOND))
                    .setReversed(false);
            static TrajectoryConfig forwardConfigSlow = new TrajectoryConfig(MIN_VELOCITY_METERS_PER_SECOND,
                    MIN_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(kDriveKinematics)
                    .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                            MAX_VELOCITY_METERS_PER_SECOND))
                    .setReversed(false);
            static TrajectoryConfig reverseConfigFast = new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND,
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(kDriveKinematics)
                    .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                            MAX_VELOCITY_METERS_PER_SECOND))
                    .setReversed(true);
            static TrajectoryConfig reverseConfigSlow = new TrajectoryConfig(MIN_VELOCITY_METERS_PER_SECOND,
                    MIN_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(kDriveKinematics)
                    .addConstraint(new SwerveDriveKinematicsConstraint(kDriveKinematics,
                            MAX_VELOCITY_METERS_PER_SECOND))
                    .setReversed(true);
    }

    public TrajectoryGenerator() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    public Trajectory getLastTrajectory() {
        return m_LastTrajectory;
    }

    public ProfiledPIDController getThetaController() {
        return thetaController;
    }

    public Trajectory getDriveTest() {
        return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
                .generateTrajectory(
                        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                new Pose2d(0, 5, Rotation2d.fromDegrees(180))),
                        TrajectoryConfigs.forwardConfigFast);
    }

    public Trajectory getPlannerTest() {
        return PathPlanner.loadPath("New Path", MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }
}