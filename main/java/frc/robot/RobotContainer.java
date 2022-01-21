// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.AutonomousTrajectories;

import java.io.IOException;

import org.frcteam2910.common.math.Rotation2;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.PS4Gamepad;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToLoadingStationCommand;
import frc.robot.commands.VisionRotateToTargetCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.DriverReadout;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final Superstructure superstructure = new Superstructure();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

  private AutonomousTrajectories autonomousTrajectories;
  private final AutonomousChooser autonomousChooser;

  private final DriverReadout driverReadout;

  private final PS4Gamepad primaryController = new PS4Gamepad(Constants.PRIMARY_CONTROLLER_PORT);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonomousChooser = new AutonomousChooser(autonomousTrajectories);

    driverReadout = new DriverReadout(this);
    
    primaryController.getLeftXAxis().setInverted(true);
    primaryController.getRightXAxis().setInverted(true);
    primaryController.getRightXAxis().setScale(0.25);

    // Configure the button bindings
    configureButtonBindings();

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
    CommandScheduler.getInstance().registerSubsystem(visionSubsystem);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    primaryController.getBackButton().whenPressed(
      () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
    );

    // primaryController.getAButton().whileHeld(new VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem, 
    // () -> getDriveForwardAxis().get(true),
    // () -> getDriveStrafeAxis().get(true)));

    // primaryController.getLeftBumperButton().whileHeld(new DriveToLoadingStationCommand(drivetrainSubsystem, visionSubsystem, driverReadout::getSelectedLoadingBay));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(this);
  }
  //-------------------------------------------------

  private org.frcteam2910.common.robot.input.Axis getDriveForwardAxis() {
    return primaryController.getLeftYAxis();
  }

  private org.frcteam2910.common.robot.input.Axis getDriveStrafeAxis() {
    return primaryController.getLeftXAxis();
  }

  private org.frcteam2910.common.robot.input.Axis getDriveRotationAxis() {
    return primaryController.getRightXAxis() ;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }
  
  public org.frcteam2910.common.robot.input.XboxController getPrimaryController() {
    return primaryController;
  }
}
