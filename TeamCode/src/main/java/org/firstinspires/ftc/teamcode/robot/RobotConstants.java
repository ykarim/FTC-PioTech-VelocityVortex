package org.firstinspires.ftc.teamcode.robot;

public class RobotConstants {

    static String TAG = "FTC APP";

    static String frontLeftMotor = "FL";
    static String frontRightMotor = "FR";
    static String backLeftMotor = "BL";
    static String backRightMotor = "BR";

    static String intakeMotor = "INTAKE";
    static String shootMotor = "SHOOT";
    static String capMotor = "CAP";

    static String opticalSensor = "OPTICAL";

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

    public static double gamepadThreshold = 0.15;
}
