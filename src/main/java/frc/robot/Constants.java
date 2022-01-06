// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class VoltageConstraints {
                public static final double MAX_VOLTAGE = 12.0;

                public static final int angleContinuousCurrentLimit = 25;
                public static final int anglePeakCurrentLimit = 40;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 35;
                public static final int drivePeakCurrentLimit = 60;
                public static final double drivePeakCurrentDuration = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                public static final double m_openLoopRamp = 0.25;
                public static final double m_closedLoopRamp = 0.0;
        }

        public static class MotionConstraints {
                public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
                                * SdsModuleConfigurations.MK3_STANDARD.getDriveReduction()
                                * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
                public static final double MIN_VELOCITY_METERS_PER_SECOND = 0;

                public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;
                public static final double MIN_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.5;

                public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                                / Math.hypot(GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                                GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS / 2.0);
                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_EPER_SECOND_SQUARED = 0;
        }

        public static class GeometricCoefficients {
                public static final double m_kS = 0 / 12;
                public static final double m_kV = 0 / 12;
                public static final double m_kA = 0 / 12;

                public static final double m_diameter = 0;
                public static final double m_circumfrence = m_diameter * 2 * Math.PI;

                public static final double m_driveGearRatio = (6.86 / 1.0); // 6.86:1
                public static final double m_angleGearRatio = (12.8 / 1.0); // 12.8:1

                public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
                public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase
        }

        public static class MappingPorts {
                public static final int DRIVETRAIN_PIGEON_ID = 12; // FIXME Set Pigeon ID

                public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
                public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set front left module steer motor ID
                public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set front left steer encoder ID

                public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front right drive motor ID
                public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
                public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; // FIXME Set front right steer encoder ID

                public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6; // FIXME Set back left drive motor ID
                public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; // FIXME Set back left steer motor ID
                public static final int BACK_LEFT_MODULE_STEER_ENCODER = 8; // FIXME Set back left steer encoder ID

                public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 9; // FIXME Set back right drive motor ID
                public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10; // FIXME Set back right steer motor ID
                public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set back right steer encoder ID
        }

        public static class LimelightCoefficients {
                public static final double kPLimelightController = 0.003;
                public static final double kLimelightMinCorrection = 0.001;
                public static final double kLimelightDeadZone = 0.25;

                public static final double LIMELIGHT_X_OFFSET = 0.0;
                public static final double LIMELIGHT_Y_OFFSET = 2.0;
        }

        public static class DrivetrainCoefficients {

                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                -GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                                new Translation2d(-GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS / 2,
                                                -GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS / 2));

                public static final SwerveModuleState[] ZERO_STATES = {new SwerveModuleState(), new SwerveModuleState(),
                                new SwerveModuleState(), new SwerveModuleState()};

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                MotionConstraints.MAX_VELOCITY_METERS_PER_SECOND,
                                MotionConstraints.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                public static final double kPDrive = 1;
                public static final double kIDrive = 1;
                public static final double kDDrive = 1;
                public static final double kFDrive = 1;

                public static final double kPSteer = 1;
                public static final double kISteer = 1;
                public static final double kDSteer = 1;
                public static final double kFSteer = 1;

                public static final double kPHeading = 0.01;

                public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and
                                                                                                  // set front
                public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and
                                                                                                   // set
                                                                                                   // front
                public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and
                                                                                                 // set front
                public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and
                                                                                                  // set front
        }
}