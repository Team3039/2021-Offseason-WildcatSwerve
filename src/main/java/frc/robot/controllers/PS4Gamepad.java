package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Contains functions for use with the Playstation 4 controller. Credit to these
 * guys
 * 
 * @author Joshua Lewis joshlewi14@gmail.com
 * @author Thomas Floyd tefloyd1215@att.net and
 */
public class PS4Gamepad extends Joystick {
	// Gamepad axis ports
	private static final int AXIS_LEFT_X = 0;
	private static final int AXIS_LEFT_Y = 1;
	private static final int AXIS_RIGHT_X = 2;
	private static final int AXIS_RIGHT_Y = 5;
	// private static final int AXIS_BUMPER = 4;
	private static final int AXIS_DPAD = 5;

	// Gamepad buttons
	public static final int BUTTON_X = 2;
	public static final int BUTTON_Square = 1;
	public static final int BUTTON_Circle = 3;
	public static final int BUTTON_Triangle = 4;
	private static final int BUTTON_L1 = 5;
	private static final int BUTTON_R1 = 6;
	private static final int BUTTON_L2 = 7;
	private static final int BUTTON_R2 = 8;
	private static final int BUTTON_SHARE = 9;
	private static final int BUTTON_OPTIONS = 10;
	private static final int BUTTON_L3 = 11;
	private static final int BUTTON_R3 = 12;
	private static final int BUTTON_START = 13;
	private static final int BUTTON_PAD = 14;

	public int DPAD_UP = 0;
	public int DPAD_UP_RIGHT = 45;
	public int DPAD_RIGHT = 90;
	public int DPAD_DOWN_RIGHT = 135;
	public int DPAD_DOWN = 180;
	public int DPAD_DOWN_LEFT = 225;
	public int DPAD_LEFT = 270;
	public int DPAD_UP_LEFT = 315;

	/**
	 * Constructor that creates a Joystick object.
	 */
	public PS4Gamepad(int gamepadPort) {
		super(gamepadPort);
	}

	/**
	 * Returns the X position of the left stick.
	 */
	public double getLeftXAxis() {
		return getRawAxis(AXIS_LEFT_X);
	}

	/**
	 * Returns the X position of the right stick.
	 */
	public double getRightXAxis() {
		return getRawAxis(AXIS_RIGHT_X);
	}

	/**
	 * Returns the Y position of the left stick.
	 */
	public double getLeftYAxis() {
		return getRawAxis(AXIS_LEFT_Y);
	}

	/**
	 * Returns the Y position of the right stick.
	 */
	public double getRightYAxis() {
		return getRawAxis(AXIS_RIGHT_Y);
	}

	/**
	 * Checks whether Button X is being pressed and returns true if it is.
	 */
	public boolean getButtonStateX() {
		return getRawButton(BUTTON_X);
	}

	/**
	 * Checks whether Button Circle is being pressed and returns true if it is.
	 */
	public boolean getButtonStateCircle() {
		return getRawButton(BUTTON_Circle);
	}

	/**
	 * Checks whether Button Sqaure is being pressed and returns true if it is.
	 */
	public boolean getButtonStateSquare() {
		return getRawButton(BUTTON_Square);
	}

	/**
	 * Checks whether Button Triangle is being pressed and returns true if it is.
	 */
	public boolean getButtonStateTriangle() {
		return getRawButton(BUTTON_Triangle);

	}

	public boolean getButtonStatePad() {
		return getRawButton(BUTTON_PAD);
	}

	/**
	 * Returns an object of Button A.
	 */
	public JoystickButton getButtonX() {
		return new JoystickButton(this, BUTTON_X);
	}

	/**
	 * Returns an object of Button B.
	 */
	public JoystickButton getButtonCircle() {
		return new JoystickButton(this, BUTTON_Circle);
	}

	/**
	 * Returns an object of Button X.
	 */
	public JoystickButton getButtonSquare() {
		return new JoystickButton(this, BUTTON_Square);
	}

	/**
	 * Returns an object of Button Y.
	 */
	public JoystickButton getButtonTriangle() {
		return new JoystickButton(this, BUTTON_Triangle);
	}

	public JoystickButton getButtonPad() {
		return new JoystickButton(this, BUTTON_PAD);
	}

	/**
	 * Return the DPad axis positions.
	 */
	public double getDPadX() {
		return getRawAxis(AXIS_DPAD);
	}

	/**
	 * DPad Left and Right only WPILIB cannot access the vertical axis of the
	 * Playstation 4
	 */

	public Button getDPadUp() {
		return new DPadTriggerButton(this, DPAD_UP);
	}

	public Button getDPadDown() {
		return new DPadTriggerButton(this, DPAD_DOWN);
	}

	public Button getDPadLeft() {
		return new DPadTriggerButton(this, DPAD_LEFT);
	}

	public Button getDPadRight() {
		return new DPadTriggerButton(this, DPAD_RIGHT);
	}

	/**
	 * /** Gets the state of the Start button
	 * 
	 * @return the state of the Start button
	 */
	public JoystickButton getOptionsButton() {
		return new JoystickButton(this, BUTTON_OPTIONS);
	}

	public JoystickButton getShareButton() {
		return new JoystickButton(this, BUTTON_SHARE);
	}

	public JoystickButton getStartButton() {
		return new JoystickButton(this, BUTTON_START);
	}

	/**
	 * Gets the state of the left bumper
	 * 
	 * @return the state of the left bumper
	 */
	public JoystickButton getL1() {
		return new JoystickButton(this, BUTTON_L1);
	}

	/**
	 * Gets the state of the right bumper
	 * 
	 * @return the state of the right bumper
	 */
	public JoystickButton getR1() {
		return new JoystickButton(this, BUTTON_R1);
	}

	/**
	 * Gets the state of the left stick button
	 * 
	 * @return the state of the left stick button
	 */
	public JoystickButton getL3() {
		return new JoystickButton(this, BUTTON_L3);
	}

	/**
	 * Gets the state of the right stick button
	 * 
	 * @return the state of the right stick button
	 */
	public JoystickButton getR3() {
		return new JoystickButton(this, BUTTON_R3);
	}

	public JoystickButton getL2() {
		return new JoystickButton(this, BUTTON_L2);
	}

	/**
	 * Gets the state of the right trigger
	 * 
	 * @return the state of the right trigger
	 */
	public JoystickButton getR2() {
		return new JoystickButton(this, BUTTON_R2);
	}

	public int getDpadAngle() {
		return this.getPOV();
	}

	// public Button getRightTrigger() {
	// return new AxisTriggerButton(this, Mac.AXIS_RIGHT_TRIGGER,
	// Constants.TRIGGER_TOLERANCE);
	// }
	// public Button getLeftTrigger() {
	// return new AxisTriggerButton(this, map.AXIS_LEFT_TRIGGER,
	// Constant..TRIGGER_TOLERANCE);
	// }
}

class DPadTriggerButton extends Button {

	private int buttonAngle;
	private PS4Gamepad controller;

	public DPadTriggerButton(PS4Gamepad controller, int dPadButtonAngle) {
		this.buttonAngle = dPadButtonAngle;
		this.controller = controller;
	}

	@Override
	public boolean get() {
		return controller.getDpadAngle() == buttonAngle;
	}
}