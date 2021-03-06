// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ToggleAlternateCenter;
import frc.robot.commands.ToggleFieldRelative;
import frc.robot.commands.ToggleGearing;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final static InterpolatedPS4Gamepad m_driver = new InterpolatedPS4Gamepad(0);
    // The robot's subsystems and commands are defined here...
    public final Drive m_drive = new Drive();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    public static InterpolatedPS4Gamepad getDriver() {
        return m_driver;
    }

    public static boolean inDeadZone(double axis) {
        return (axis > -0.05) && (axis < 0.05);
    }

    public static boolean isCeiling(double axis) {
        return axis >= 0.9;
    }

    public static double interpolatedLeftYAxis() {
        return m_driver.interpolatedLeftYAxis();
    }

    public static double interpolatedLeftXAxis() {
        return m_driver.interpolatedLeftXAxis();
    }

    public static double interpolatedRightXAxis() {
        return m_driver.interpolatedRightXAxis();
    }

    public static void outputTelemetry(String telemetry) {
        System.out.println("System Telemetry :: " + telemetry);
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

        Button driverX = getDriver().getButtonX();
        driverX.toggleWhenPressed(new ToggleFieldRelative());

        Button driverR1 = getDriver().getR1();
        driverR1.toggleWhenPressed(new ToggleGearing(true));

        Button driverL1 = getDriver().getL1();
        driverL1.toggleWhenPressed(new ToggleAlternateCenter(true));
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
}
