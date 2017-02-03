package org.firstinspires.ftc.teamcode;

class RobotConstants {

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
    static final double distFromCenterToWheel = Math.sqrt(162);
}
