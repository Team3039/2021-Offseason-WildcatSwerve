// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlMode;

public class ToggleFieldRelative extends CommandBase {
    /**
     * Creates a new ToggleFieldRelative.
     */
    public ToggleFieldRelative() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Drive.getInstance().setControlMode(DriveControlMode.JOYSTICK_ROBOT_CENTRIC);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Drive.getInstance().setControlMode(DriveControlMode.JOYSTICK_FIELD_CENTRIC);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}