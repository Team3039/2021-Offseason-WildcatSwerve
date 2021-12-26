package frc.robot.util;

import static frc.robot.Constants.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.angleGearRatio;
import static frc.robot.Constants.driveGearRatio;
import static frc.robot.Constants.m_circumfrence;
import static frc.robot.Constants.m_kA;
import static frc.robot.Constants.m_kS;
import static frc.robot.Constants.m_kV;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.subsystems.Drive;

public class SwerveModuleClosedLoop {
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(m_kS, m_kV, m_kA);

    public SwerveModuleClosedLoop(TalonFX driveMotor, TalonFX steerMotor, CANCoder absEncoder, double steerOffset) {
        angleOffset = steerOffset;

        mDriveMotor = driveMotor;
        mAngleMotor = steerMotor;
        angleEncoder = absEncoder;

        configDriveMotor();
        configAngleMotor();
        configAngleEncoder();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = Drive.optimize(desiredState, getState().angle); // Custom optimize command, since default WPILib
                                                                       // optimize assumes continuous controller which
                                                                       // CTRE is not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }

        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, m_circumfrence,
                    driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_VELOCITY_METERS_PER_SECOND * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, angleGearRatio));
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(CTREConfigs.getInstance().swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(CTREConfigs.getInstance().swerveAngleFXConfig);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(CTREConfigs.getInstance().swerveDriveFXConfig);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), m_circumfrence,
                driveGearRatio);
        Rotation2d angle = Rotation2d
                .fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
}