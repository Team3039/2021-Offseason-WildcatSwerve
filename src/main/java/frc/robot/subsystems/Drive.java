// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.SwerveModuleClosedLoop;

import java.util.List;

import static frc.robot.Constants.DrivetrainCoefficients.*;
import static frc.robot.Constants.GeometricCoefficients.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.GeometricCoefficients.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.MappingPorts.*;
import static frc.robot.Constants.MotionConstraints.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.MotionConstraints.MAX_VELOCITY_METERS_PER_SECOND;

public class Drive extends SubsystemBase {

    private final static Drive INSTANCE = new Drive();

    private final SwerveModuleClosedLoop m_frontLeftModule;
    private final SwerveModuleClosedLoop m_frontRightModule;
    private final SwerveModuleClosedLoop m_backLeftModule;
    private final SwerveModuleClosedLoop m_backRightModule;

    private final List<SwerveModuleClosedLoop> m_modulesClosedLoop;

    private final PigeonIMU m_pigeon;

    TalonFX m_frontLeftDrive;
    TalonFX m_frontRightDrive;
    TalonFX m_backLeftDrive;
    TalonFX m_backRightDrive;

    TalonFX m_frontLeftSteer;
    TalonFX m_frontRightSteer;
    TalonFX m_backLeftSteer;
    TalonFX m_backRightSteer;

    CANCoder m_frontLeftCanCoder;
    CANCoder m_frontRightCanCoder;
    CANCoder m_backLeftCanCoder;
    CANCoder m_backRightCanCoder;

    double m_storedYaw;
    double yaw;
    double m_yawCorrection;
    boolean m_isHighGear;
    boolean m_alternateCenter;
    boolean m_isOpenLoop;

    SwerveDriveOdometry m_odometry;

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK_FIELD_CENTRIC;
    private ChassisSpeeds m_chassisSpeeds;
    private SwerveModuleState[] m_ModuleStates;

    public Drive() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftDrive = new TalonFX(FRONT_LEFT_MODULE_DRIVE_MOTOR);
        m_frontLeftSteer = new TalonFX(FRONT_LEFT_MODULE_STEER_MOTOR);
        m_frontLeftCanCoder = new CANCoder(FRONT_LEFT_MODULE_STEER_ENCODER);

        m_frontRightDrive = new TalonFX(FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        m_frontRightSteer = new TalonFX(FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        m_frontRightCanCoder = new CANCoder(FRONT_RIGHT_MODULE_STEER_ENCODER);

        m_backLeftDrive = new TalonFX(BACK_LEFT_MODULE_DRIVE_MOTOR);
        m_backLeftSteer = new TalonFX(BACK_LEFT_MODULE_STEER_MOTOR);
        m_backLeftCanCoder = new CANCoder(BACK_LEFT_MODULE_STEER_ENCODER);

        m_backRightDrive = new TalonFX(BACK_RIGHT_MODULE_DRIVE_MOTOR);
        m_backRightSteer = new TalonFX(BACK_RIGHT_MODULE_STEER_MOTOR);
        m_backRightCanCoder = new CANCoder(BACK_RIGHT_MODULE_STEER_ENCODER);

        m_frontLeftModule = new SwerveModuleClosedLoop(m_frontLeftDrive, m_frontLeftSteer, m_frontLeftCanCoder,
                FRONT_LEFT_MODULE_STEER_OFFSET);
        m_frontRightModule = new SwerveModuleClosedLoop(m_frontRightDrive, m_frontRightSteer, m_frontRightCanCoder,
                FRONT_RIGHT_MODULE_STEER_OFFSET);
        m_backLeftModule = new SwerveModuleClosedLoop(m_backLeftDrive, m_backLeftSteer, m_backLeftCanCoder,
                BACK_LEFT_MODULE_STEER_OFFSET);
        m_backRightModule = new SwerveModuleClosedLoop(m_backRightDrive, m_backRightSteer, m_backRightCanCoder,
                BACK_RIGHT_MODULE_STEER_OFFSET);

        m_modulesClosedLoop = List.of(m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule);

        m_chassisSpeeds = new ChassisSpeeds();
        m_ModuleStates = ZERO_STATES;

        m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

        m_odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(getPigeon().getFusedHeading()));

        m_isHighGear = false;
        m_alternateCenter = false;
        m_isOpenLoop = false;
    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    public synchronized DriveControlMode getControlMode() {
        return driveControlMode;
    }

    public synchronized void setControlMode(DriveControlMode controlMode) {
        this.driveControlMode = controlMode;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (isOpenLoop()) {
            m_frontLeftModule.setDesiredState(desiredStates[0],
                    true);
            m_frontRightModule.setDesiredState(desiredStates[1], true);
            m_backLeftModule.setDesiredState(desiredStates[2], true);
            m_backRightModule.setDesiredState(desiredStates[3], true);
        }
        else {
            for (int i = 0; i <= 3; i++) {
                m_modulesClosedLoop.get(i).setDesiredState(desiredStates[i], false);
            }
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        m_chassisSpeeds = chassisSpeeds;
        m_isOpenLoop = isOpenLoop;

        gearingConfig();
        alternateCenterConfig();

        setModuleStates(m_ModuleStates);
    }

    public void driveManualFieldCentric() {
        updatedStoredYaw();

        drive(ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.interpolatedLeftYAxis(),
                RobotContainer.interpolatedLeftXAxis(), RobotContainer.interpolatedRightXAxis() + m_yawCorrection,
                getGyroscopeRotation()), true);
    }

    public void driveManualRobotCentric() {
        updatedStoredYaw();

        drive(new ChassisSpeeds(RobotContainer.interpolatedLeftYAxis(), RobotContainer.interpolatedLeftXAxis(),
                RobotContainer.interpolatedRightXAxis()), true);
    }

    public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i <= 3; i++) {
            m_modulesClosedLoop.get(i).setDesiredState(desiredStates[i], false);
        }
    }

    public void trackTarget(boolean lockX, boolean lockY, boolean lockTheta) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                lockX ? ClosedLoopFeedback.calculateHomingOutputTargetX() : RobotContainer.interpolatedLeftXAxis(),
                lockY ? ClosedLoopFeedback.calculateHomingOutputTargetY() : RobotContainer.interpolatedLeftYAxis(),
                lockTheta ? ClosedLoopFeedback.calculateHomingOutputTargetX() : RobotContainer.interpolatedRightXAxis(),
                getGyroscopeRotation()), false);
    }

    double calcYawStraight(double targetAngle, double currentAngle) {
        double errorAngle = (targetAngle - currentAngle) % 360;
        double correction = errorAngle * kPHeading;
        return -1 * correction;
    }

    public void gearingConfig() {
        if (m_isHighGear) {
            m_chassisSpeeds.vxMetersPerSecond *= MAX_VELOCITY_METERS_PER_SECOND;
            m_chassisSpeeds.vyMetersPerSecond *= MAX_VELOCITY_METERS_PER_SECOND;
            m_chassisSpeeds.omegaRadiansPerSecond *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        } else {
            m_chassisSpeeds.vxMetersPerSecond *= (MAX_VELOCITY_METERS_PER_SECOND - 1);
            m_chassisSpeeds.vyMetersPerSecond *= (MAX_VELOCITY_METERS_PER_SECOND - 1);
            m_chassisSpeeds.omegaRadiansPerSecond *= (MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND - 1);
        }
    }

    public void alternateCenterConfig() {
        if (m_alternateCenter) {
            m_ModuleStates = kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds,
                    new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2, -DRIVETRAIN_TRACKWIDTH_METERS / 2));
        } else {
            m_ModuleStates = kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
        }
    }

    public SwerveModuleState getFrontLeftState() {
        return m_frontLeftModule.getState();
    }

    public SwerveModuleState getFrontRightState() {
        return m_frontRightModule.getState();
    }

    public SwerveModuleState getBackLeftState() {
        return m_backLeftModule.getState();
    }

    public SwerveModuleState getBackRightState() {
        return m_backRightModule.getState();
    }

    public PigeonIMU getPigeon() {
        return m_pigeon;
    }

    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getPigeon().getFusedHeading());
    }

    public void updateOdometry() {
        m_odometry.update(getRotation2d(), getFrontLeftState(), getBackLeftState(), getFrontRightState(),
                getBackRightState());
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, getRotation2d());
    }

    public boolean isHighGear() {
        return m_isHighGear;
    }

    public void setHighGear(boolean gear) {
        m_isHighGear = gear;
    }

    public boolean isAlternateCenter() {
        return m_alternateCenter;
    }

    public void setAlternateCenter(boolean altCenter) {
        m_alternateCenter = altCenter;
    }

    public boolean isOpenLoop() {
        return m_isOpenLoop;
    }

    public void setOpenLoop(boolean isOpenLoop) {
        m_isOpenLoop = isOpenLoop;
    }

    public void updatedStoredYaw() {
//        if (!RobotContainer.inDeadZone(RobotContainer.getDriver().getRightXAxis())) {
//            m_storedYaw = yaw;
//            m_yawCorrection = 0;
//        } else {
//            if (Math.abs(RobotContainer.getDriver().getLeftYAxis()) > 0
//                    || Math.abs(RobotContainer.getDriver().getLeftXAxis()) > 0) {
//                m_yawCorrection = calcYawStraight(m_storedYaw, yaw);
//            }
//        }
        m_yawCorrection = 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        yaw = m_pigeon.getFusedHeading();
        updateOdometry();

        synchronized (Drive.this) {
            DriveControlMode currentControlMode = getControlMode();
            switch (currentControlMode) {
                case JOYSTICK_FIELD_CENTRIC:
                    driveManualFieldCentric();
                    break;
                case JOYSTICK_ROBOT_CENTRIC:
                    driveManualRobotCentric();
                    break;
                case HOMING:
                    trackTarget(true, true, false);
                    break;
                case PATH_FOLLOWING:
                    break;
                default:
                    System.out.println("Unknown drive control mode: " + currentControlMode);
            }
        }
    }

    public enum DriveControlMode {
        JOYSTICK_FIELD_CENTRIC, JOYSTICK_ROBOT_CENTRIC, HOMING, PATH_FOLLOWING
    }
}