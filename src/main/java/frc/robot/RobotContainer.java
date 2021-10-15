// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SetHighGear;
import frc.robot.commands.ToggleFieldRelative;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static Drive m_drive = new Drive();

  public final static InterpolatedPS4Gamepad m_driver = new InterpolatedPS4Gamepad(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    Button driverPadButton = getDriver().getButtonPad();
    driverPadButton.whenPressed(new ZeroGyroscope());

    Button driverL1 = getDriver().getL1();
    driverL1.toggleWhenPressed(new ToggleFieldRelative());

    Button driverR1 = getDriver().getR1();
    driverR1.toggleWhenPressed(new SetHighGear());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public static InterpolatedPS4Gamepad getDriver() {
    return m_driver;
  }

  public static boolean inDeadZone(double axis) {
    if ((axis > -0.05) && (axis < 0.05))
      return true;
    return false;
  }

  public static boolean isCeiling(double axis) {
    if (axis >= 0.9)
      return true;
    return false;
  }

  public static double interpolatedLeftYAxis() {
    if (RobotContainer.inDeadZone(getDriver().getLeftYAxis()))
      return 0.0;
    if (RobotContainer.isCeiling(getDriver().getLeftYAxis()))
      return 1.0;
    if (Drive.getInstance().isHighGear())
      return Math.sqrt(getDriver().getLeftYAxis());
    else
      return getDriver().getLeftYAxis() * getDriver().getLeftYAxis();
  }

  public static double interpolatedLeftXAxis() {
    if (RobotContainer.inDeadZone(getDriver().getLeftXAxis()))
      return 0.0;
    if (RobotContainer.isCeiling(getDriver().getLeftXAxis()))
      return 1.0;
    if (Drive.getInstance().isHighGear())
      return Math.sqrt(getDriver().getLeftXAxis());
    else
      return getDriver().getLeftXAxis() * getDriver().getLeftXAxis();
  }

  public static double interpolatedRightXAxis() {
    if (RobotContainer.inDeadZone(getDriver().getRightXAxis()))
      return 0.0;
    if (RobotContainer.isCeiling(getDriver().getRightXAxis()))
      return 1.0;
    if (Drive.getInstance().isHighGear())
      return Math.sqrt(getDriver().getRightXAxis());
    else
      return getDriver().getRightXAxis() * getDriver().getRightXAxis();
  }

  public static void outputTelemetry(String telemetry) {
    if (!telemetry.equals(null))
      System.out.println("System Telemetry :: " + telemetry);
  }
}