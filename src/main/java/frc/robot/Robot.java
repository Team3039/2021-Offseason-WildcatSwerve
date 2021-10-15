// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.routines.DoNothing;
import frc.robot.auto.routines.TestAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Timer m_timer = new Timer();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Drive drive = Drive.getInstance();

  public SendableChooser<Command> autonTaskChooser;

  public Field2d m_Field2d;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    drive.resetOdometry(new Pose2d());
    TrajectoryGenerator.getInstance().thetaController.enableContinuousInput(-Math.PI, Math.PI);

    autonTaskChooser = new SendableChooser<>();

    autonTaskChooser.addOption("Test Auto", new TestAuto());
    autonTaskChooser.setDefaultOption("Test Auto", new TestAuto());

    autonTaskChooser.addOption("Do Nothing", new DoNothing());

    SmartDashboard.putData("Autonomous Selector", autonTaskChooser);

    m_Field2d = new Field2d();
    SmartDashboard.putData("Field", m_Field2d);

    // UsbCamera usbCamera = CameraServer.getInstance().startAutomaticCapture();
    // usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Time Remaining", Timer.getMatchTime());
    SmartDashboard.updateValues();

    m_Field2d.setRobotPose(RobotContainer.m_drive.getPose());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    RobotContainer.m_drive.setControlMode(DriveControlMode.PATH_FOLLOWING);

    m_autonomousCommand = autonTaskChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_timer.start();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    drive.setControlMode(DriveControlMode.JOYSTICK_FIELD_CENTRIC);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
