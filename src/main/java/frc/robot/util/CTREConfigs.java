package frc.robot.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import static frc.robot.Constants.*;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public static CTREConfigs INSTANCE = new CTREConfigs();

    public static CTREConfigs getInstance() {
        return INSTANCE;
    }

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(angleEnableCurrentLimit,
                angleContinuousCurrentLimit, anglePeakCurrentLimit, anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = kPSteer;
        swerveAngleFXConfig.slot0.kI = kISteer;
        swerveAngleFXConfig.slot0.kD = kDSteer;
        swerveAngleFXConfig.slot0.kF = kFSteer;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(driveEnableCurrentLimit,
                driveContinuousCurrentLimit, drivePeakCurrentLimit, drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = kPDrive;
        swerveDriveFXConfig.slot0.kI = kIDrive;
        swerveDriveFXConfig.slot0.kD = kDDrive;
        swerveDriveFXConfig.slot0.kF = kFDrive;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = closedLoopRamp;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        swerveCanCoderConfig.sensorDirection = true;
    }

}