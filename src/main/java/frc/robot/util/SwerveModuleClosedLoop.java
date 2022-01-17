package frc.robot.util;

import static frc.robot.Constants.GeometricCoefficients.m_angleGearRatio;
import static frc.robot.Constants.GeometricCoefficients.m_circumference;
import static frc.robot.Constants.GeometricCoefficients.m_driveGearRatio;
import static frc.robot.Constants.GeometricCoefficients.m_kA;
import static frc.robot.Constants.GeometricCoefficients.m_kS;
import static frc.robot.Constants.GeometricCoefficients.m_kV;
import static frc.robot.Constants.MotionConstraints.MAX_VELOCITY_METERS_PER_SECOND;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

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

        desiredState = optimize(desiredState, getState().angle); // Custom optimize command, since default WPILib
                                                                       // optimize assumes continuous controller which
                                                                       // CTRE is not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }

        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, m_circumference,
                    m_driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_VELOCITY_METERS_PER_SECOND * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, m_angleGearRatio));
        lastAngle = angle;
    }

    private void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, m_angleGearRatio);
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

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
          targetSpeed = -targetSpeed;
          targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
        } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
          newAngle += 360;
        }
        while (newAngle > upperBound) {
          newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
          newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
        }
        return newAngle;
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), m_circumference,
                m_driveGearRatio);
        Rotation2d angle = Rotation2d
                .fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), m_angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
}