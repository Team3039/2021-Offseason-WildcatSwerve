package frc.robot;

public class Constants {
    //CAN IDs for swerve modules
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 6;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 9;

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 7;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 10;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 5;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 11;

    public static final int PIGEON_PORT = 12;

    //----------------------------------------------------------------------------------------------------------------------------------

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(181.5);//312.7
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(21.8);//18.7
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(34.1);//212.5
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(231.7);//200

    public static final int PRIMARY_CONTROLLER_PORT = 0;


    public static final int PRESSURE_SENSOR_PORT = 0;
}
