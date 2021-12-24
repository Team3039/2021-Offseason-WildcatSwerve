// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.*;

import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {

  private final static Drive INSTANCE = new Drive();

  public static Drive getInstance() {
    return INSTANCE;
  }

  public enum DriveControlMode {
    JOYSTICK_FIELD_CENTRIC, JOYSTICK_ROBOT_CENTRIC, HOMING, PATH_FOLLOWING
  }

  private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK_FIELD_CENTRIC;

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds driveChassisSpeeds;

  private final PigeonIMU m_pigeon;

  double storedYaw;
  double yaw;
  double yawCorrection;

  boolean isHighGear;

  SwerveDriveOdometry m_odometry;

  public Drive() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR,
        FRONT_LEFT_MODULE_STEER_ENCODER, FRONT_LEFT_MODULE_STEER_OFFSET);

    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER, FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER, BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
        Mk3SwerveModuleHelper.GearRatio.STANDARD, BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER, BACK_RIGHT_MODULE_STEER_OFFSET);

    m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    m_odometry = new SwerveDriveOdometry(Constants.kDriveKinematics,
        Rotation2d.fromDegrees(getPigeon().getFusedHeading()));

    isHighGear = true;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);

    m_frontLeftModule.set(
        desiredStates[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        desiredStates[0].angle.getRadians());
    m_frontRightModule.set(
        desiredStates[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        desiredStates[1].angle.getRadians());
    m_backLeftModule.set(
        desiredStates[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        desiredStates[2].angle.getRadians());
    m_backRightModule.set(
        desiredStates[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE,
        desiredStates[3].angle.getRadians());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {

    driveChassisSpeeds = chassisSpeeds;

    if (isHighGear()) {
      driveChassisSpeeds.vxMetersPerSecond *= Constants.MAX_VELOCITY_METERS_PER_SECOND;
      driveChassisSpeeds.vyMetersPerSecond *= Constants.MAX_VELOCITY_METERS_PER_SECOND;
      driveChassisSpeeds.omegaRadiansPerSecond *= Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    else {
      driveChassisSpeeds.vxMetersPerSecond *= (Constants.MAX_VELOCITY_METERS_PER_SECOND - 1);
      driveChassisSpeeds.vyMetersPerSecond *= (Constants.MAX_VELOCITY_METERS_PER_SECOND - 1);
      driveChassisSpeeds.omegaRadiansPerSecond *= (Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND - 1);
    }

    SwerveModuleState[] states = Constants.kDriveKinematics.toSwerveModuleStates(driveChassisSpeeds);

    setModuleStates(states);
  }

  public void driveManualFieldCentric() {
    if (!RobotContainer.inDeadZone(RobotContainer.getDriver().getRightXAxis())) {
      storedYaw = yaw;
      yawCorrection = 0;
    }

    else {
      if (Math.abs(RobotContainer.getDriver().getLeftYAxis()) > 0
          || Math.abs(RobotContainer.getDriver().getLeftXAxis()) > 0) {
        yawCorrection = calcYawStraight(storedYaw, yaw, Constants.kPHeading);
      }
    }
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.interpolatedLeftYAxis(),
        RobotContainer.interpolatedLeftXAxis(), RobotContainer.interpolatedRightXAxis() + yawCorrection, getGyroscopeRotation()));
  }

  public void driveManualRobotCentric() {
    if (!RobotContainer.inDeadZone(RobotContainer.getDriver().getRightXAxis())) {
      storedYaw = yaw;
      yawCorrection = 0;
    }

    else {
      if (Math.abs(RobotContainer.getDriver().getLeftYAxis()) > 0
          || Math.abs(RobotContainer.getDriver().getLeftXAxis()) > 0) {
        yawCorrection = calcYawStraight(storedYaw, yaw, Constants.kPHeading);
      }
    }
    drive(new ChassisSpeeds(RobotContainer.interpolatedLeftYAxis(), RobotContainer.interpolatedLeftXAxis(),
        RobotContainer.interpolatedRightXAxis()));
  }

  public void trackTarget(boolean lockX, boolean lockY, boolean lockTheta) {
    if (lockX && lockY && !lockTheta) {
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(ClosedLoopFeedback.calculateHomingOutputTargetY(),
          ClosedLoopFeedback.calculateHomingOutputTargetX(), RobotContainer.interpolatedRightXAxis(),
          getGyroscopeRotation()));
    }
    if (!lockX && lockY && lockTheta) {
      drive(ChassisSpeeds.fromFieldRelativeSpeeds(RobotContainer.interpolatedLeftYAxis(),
          ClosedLoopFeedback.calculateHomingOutputTargetX(), ClosedLoopFeedback.calculateHomingOutputTargetX(),
          getGyroscopeRotation()));
    }
  }

  double calcYawStraight(double targetAngle, double currentAngle, double kP) {
    double errorAngle = (targetAngle - currentAngle) % 360;
    double correction = errorAngle * kP;
    return -1 * correction;
  }

  public synchronized DriveControlMode getControlMode() {
    return driveControlMode;
  }

  public synchronized void setControlMode(DriveControlMode controlMode) {
    this.driveControlMode = controlMode;
  }

  // TODO: Make sure getSteerAngle() returns a radian value
  public SwerveModuleState getFrontLeftState() {
    return new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
        new Rotation2d(m_frontLeftModule.getSteerAngle()));
  }

  // TODO: Make sure getSteerAngle() returns a radian value
  public SwerveModuleState getFrontRightState() {
    return new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
        new Rotation2d(m_frontRightModule.getSteerAngle()));
  }

  // TODO: Make sure getSteerAngle() returns a radian value
  public SwerveModuleState getBackLeftState() {
    return new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle()));
  }

  // TODO: Make sure getSteerAngle() returns a radian value
  public SwerveModuleState getBackRightState() {
    return new SwerveModuleState(m_backRightModule.getDriveVelocity(),
        new Rotation2d(m_backRightModule.getSteerAngle()));
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
    return isHighGear;
  }

  public void setHighGear(boolean gear) {
    isHighGear = gear;
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
}