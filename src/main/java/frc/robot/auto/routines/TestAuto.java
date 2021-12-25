// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.ResetOdometry;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.auto.commands.sequences.ResetRamsete;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetRamsete(),
      new SwerveControllerCommand(
            TrajectoryGenerator.getInstance().getDriveTest(),
            RobotContainer.m_drive::getPose,
            Constants.kDriveKinematics,
            new PIDController(Constants.kPXController, 0, 0),
            new PIDController(Constants.kPYController, 0, 0),
            TrajectoryGenerator.getInstance().getThetaController(),
            RobotContainer.m_drive::setModuleStates,
            RobotContainer.m_drive
            ),
      new ResetRamsete()
    );
  }
}