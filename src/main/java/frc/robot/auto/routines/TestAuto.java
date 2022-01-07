// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import static frc.robot.Constants.DrivetrainCoefficients.kDriveKinematics;
import static frc.robot.Constants.DrivetrainCoefficients.kPXController;
import static frc.robot.Constants.DrivetrainCoefficients.kPYController;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.sequences.ResetTrajectory;
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
      new ResetTrajectory(),
      new SwerveControllerCommand(
            TrajectoryGenerator.getInstance().getDriveTest(),
            Drive.getInstance()::getPose,
            kDriveKinematics,
            new PIDController(kPXController, 0, 0),
            new PIDController(kPYController, 0, 0),
            TrajectoryGenerator.getInstance().getThetaController(),
            Drive.getInstance()::setModuleStatesClosedLoop,
            Drive.getInstance()
            ),
      new ResetTrajectory()
    );
  }
}