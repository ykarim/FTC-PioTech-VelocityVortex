package org.firstinspires.ftc.teamcode.Robot;

public class RobotConstants {

    static String TAG = "FTC APP";
    public static final String teleOpTag = "Tele-Op : ";
    public static final String autoOpTag = "Autonomous : ";

    static String frontLeftMotor = "FL";
    static String frontRightMotor = "FR";
    static String backLeftMotor = "BL";
    static String backRightMotor = "BR";

    static String intakeMotor = "INTAKE";
    static String shootMotor = "SHOOT";
    static String capMotor = "CAP";

    static String opticalSensor = "OPTICAL";
    static String voltageSensor = "Motor Controller 1";
    static String ultrasonicSensorLeft = "ULTRALEFT";
    static String ultrasonicSensorRight = "ULTRARIGHT";

    static final int ENCODER_TICKS_PER_REV = 1120;
    static final double WHEEL_DIAMETER = 3.0;
    static final double     DRIVE_GEAR_REDUCTION = 1.0 ;
    static final double INCHES_PER_TICK = (ENCODER_TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);
    public static final double distFromCenterToWheel = Math.sqrt(162);

    public static double moveSpeed = 1.0;
    public static double rotateSpeed = 0.5;
    public static double intakeSpeed = 1.0;
    public static double shootSpeed = 1.0;
    public static double capSpeed = 1.0;

    public static double MAX_MOTOR_PWR = 1.0;
    public static double MIN_MOTOR_PWR = 0.0;

    public static boolean inverted = false;

    public static double shotWaitPeriod = 2; //seconds to wait before shooting ball
    public static double intakeWaitPeriod = 5;

    public static double gamepadThreshold = 0.15;

    public static final double whiteLineValue = 0.5;
    public static final double perfectWhiteLineValue = 0.8;

    static final double sensorWallOffset = 3;
}
