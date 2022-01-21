package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;


public class Superstructure {
    private final AnalogInput pressureSensor;

    public Superstructure() {
        pressureSensor = new AnalogInput(Constants.PRESSURE_SENSOR_PORT);
    }

    public double getCurrentPressure() {
        double voltage = pressureSensor.getAverageVoltage();
        double busVoltage = RobotController.getVoltage5V();

        return 250.0 * (voltage / busVoltage) - 25.0;
    }
}
