package frc.robot.controllers;

import frc.robot.RobotContainer;
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
        if (RobotContainer.inDeadZone(RobotContainer.getDriver().getLeftYAxis()))
         return rComponent.applyAsDouble(0.0);
        if (RobotContainer.isCeiling(RobotContainer.getDriver().getLeftYAxis()))
         return rComponent.applyAsDouble(1.0);
        if (Drive.getInstance().isHighGear()) {
         if (RobotContainer.getDriver().getLeftYAxis() < 0) {
          return rComponent.applyAsDouble(Math.tanh(RobotContainer.getDriver().getLeftYAxis() * 2));
        }
        return rComponent.applyAsDouble(-(Math.tanh(RobotContainer.getDriver().getLeftYAxis() * 2))); 
      }
       else {
        if (RobotContainer.getDriver().getLeftYAxis() < 0) {
          return rComponent.applyAsDouble(RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis());
        }
        return rComponent.applyAsDouble(-(RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis() * RobotContainer.getDriver().getLeftYAxis()));
      }
    }

    public double interpolatedLeftXAxis() {
        if (RobotContainer.inDeadZone(RobotContainer.getDriver().getLeftXAxis()))
        return rComponent.applyAsDouble(0.0);
        if (RobotContainer.isCeiling(RobotContainer.getDriver().getLeftXAxis()))
         return rComponent.applyAsDouble(1.0);
        if (Drive.getInstance().isHighGear()) {
         if (RobotContainer.getDriver().getLeftXAxis() > 0) {
          return rComponent.applyAsDouble(Math.tanh(RobotContainer.getDriver().getLeftXAxis() * 2));
      }
      return rComponent.applyAsDouble(-(Math.tanh(RobotContainer.getDriver().getLeftXAxis() * 2)));
    }
     else {
      if (RobotContainer.getDriver().getLeftXAxis() > 0) {
        return rComponent.applyAsDouble(RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis());
      }
      return rComponent.applyAsDouble(-(RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis() * RobotContainer.getDriver().getLeftXAxis()));
    }
    }


    public double interpolatedRightXAxis() {
       if (RobotContainer.inDeadZone(RobotContainer.getDriver().getRightXAxis()))
      return rComponent.applyAsDouble(0.0);
    if (RobotContainer.isCeiling(RobotContainer.getDriver().getRightXAxis()))
      return rComponent.applyAsDouble(1.0);
    if (Drive.getInstance().isHighGear()) {
        if (RobotContainer.getDriver().getRightXAxis() > 0) {
          return rComponent.applyAsDouble(Math.tanh(RobotContainer.getDriver().getRightXAxis() * 2));
        }
        return rComponent.applyAsDouble(-(Math.tanh(RobotContainer.getDriver().getRightXAxis() * 2)));
      }
  
      else {
        if (RobotContainer.getDriver().getRightXAxis() > 0) {
          return rComponent.applyAsDouble(RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis());
        }
        return rComponent.applyAsDouble(-(RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis() * RobotContainer.getDriver().getRightXAxis()));
    
    
    }
}
}