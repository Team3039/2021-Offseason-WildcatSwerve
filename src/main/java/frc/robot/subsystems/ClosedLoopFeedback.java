// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClosedLoopFeedback extends SubsystemBase {
  /** Creates a new ClosedLoopFeedback. */
  public static double limelightX;
  public static double limelightY;
  public static double limelightArea;
  public static double limelightFound;

  public ClosedLoopFeedback() {
  }

  public static double calculateHomingOutputTargetX() {
    if (!isTargetValid())
      return 0.0;

    double correction = limelightX * Constants.kPLimelightController;
    if (correction < Constants.kLimelightMinCorrection)
      correction = Math.copySign(Constants.kLimelightMinCorrection, correction);
    if (Math.abs(limelightX) < Constants.kLimelightDeadZone)
      correction = 0;
    return correction;
  }

  public static double calculateHomingOutputTargetY() {
    if (!isTargetValid())
    return 0.0;

    double targetY = limelightY + Constants.LIMELIGHT_Y_OFFSET;

    double correction = targetY * Constants.kPLimelightController;
    if (correction < Constants.kLimelightMinCorrection)
      correction = Math.copySign(Constants.kLimelightMinCorrection, correction);
    if (Math.abs(limelightY) < Constants.kLimelightDeadZone)
      correction = 0;
    return correction;
  }

  public static boolean isTargetValid() {
    if (limelightFound == 0)
      return false;
    return true;
  }

  @Override
  public void periodic() {
    limelightX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    limelightY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    limelightArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    limelightFound = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }
}
