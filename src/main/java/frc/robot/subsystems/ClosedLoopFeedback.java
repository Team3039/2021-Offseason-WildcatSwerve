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
  public final static ClosedLoopFeedback INSTANCE = new ClosedLoopFeedback();

  public static ClosedLoopFeedback getInstance() {
    return INSTANCE;
  }

  public static double limelightX;
  public static double limelightY;
  public static double limelightArea;
  public static double limelightFound;

  public static double correctionX;
  public static double correctionY;
  
  public static double errorX;
  public static double errorY;

  public ClosedLoopFeedback() {
  }

  public static double calculateHomingOutputTargetX() {
    if (!isTargetValid())
      return 0.0;

    errorX = limelightX - Constants.LIMELIGHT_X_OFFSET;
    correctionX = errorX * Constants.kPLimelightController;

    if (correctionX < Constants.kLimelightMinCorrection)
      correctionX = Math.copySign(Constants.kLimelightMinCorrection, correctionX);
    if (Math.abs(limelightX) < Constants.kLimelightDeadZone)
      correctionX = 0;

    return correctionX;
  }

  public static double calculateHomingOutputTargetY() {
    if (!isTargetValid())
      return 0.0;

    errorY = limelightY - Constants.LIMELIGHT_Y_OFFSET;
    correctionY = errorY * Constants.kPLimelightController;

    if (correctionY < Constants.kLimelightMinCorrection)
      correctionY = Math.copySign(Constants.kLimelightMinCorrection, correctionY);
    if (Math.abs(limelightY) < Constants.kLimelightDeadZone)
      correctionY = 0;

    return correctionY;
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
