// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

import static frc.robot.Constants.DrivetrainCoefficients.ZERO_STATES;

public class StopTrajectory extends CommandBase {
    /**
     * Creates a new StopTrajectory.
     */
    public StopTrajectory() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.outputTelemetry("Stopping Trajectory");
        Drive.getInstance().setModuleStates(ZERO_STATES);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.outputTelemetry("Trajectory Stopped");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
