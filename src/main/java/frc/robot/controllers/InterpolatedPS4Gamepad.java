package frc.robot.controllers;

import frc.robot.subsystems.Drive;

public class InterpolatedPS4Gamepad extends PS4Gamepad {

    static double deadZoneThreshold;
    static double fullThrottleThreshold;

    static RampComponent rComponent;

    public InterpolatedPS4Gamepad(int port) {
        super(port);

        deadZoneThreshold = 0.05;
        fullThrottleThreshold = 0.9;

        rComponent = new RampComponent(1, 0.25);
    }

    public static boolean inDeadZone(double axis) {
        if ((axis > -deadZoneThreshold) && (axis < deadZoneThreshold))
            return true;
        return false;
    }

    public static boolean isCeiling(double axis) {
        if (axis <= -fullThrottleThreshold || axis >= fullThrottleThreshold)
            return true;
        return false;
    }

    public double interpolatedLeftYAxis() {
        if (inDeadZone(getLeftYAxis()))
            return 0.0;
        if (isCeiling(getLeftYAxis()))
            return 1.0;
        if (Drive.getInstance().isHighGear())
            return rComponent.applyAsDouble(Math.sqrt(getLeftYAxis()));
        else
            return rComponent.applyAsDouble(getLeftYAxis() * getLeftYAxis());
    }

    public double interpolatedLeftXAxis() {
        if (inDeadZone(getLeftXAxis()))
            return 0.0;
        if (isCeiling(getLeftXAxis()))
            return 1.0;
        if (Drive.getInstance().isHighGear())
            return rComponent.applyAsDouble(Math.sqrt(getLeftXAxis()));
        else
            return rComponent.applyAsDouble(getLeftXAxis() * getLeftXAxis());
    }

    public double interpolatedRightXAxis() {
        if (inDeadZone(getRightXAxis()))
            return 0.0;
        if (isCeiling(getRightXAxis()))
            return 1.0;
        if (Drive.getInstance().isHighGear())
            return rComponent.applyAsDouble(Math.sqrt(getRightXAxis()));
        else
            return rComponent.applyAsDouble(getRightXAxis() * getRightXAxis());
    }
}
