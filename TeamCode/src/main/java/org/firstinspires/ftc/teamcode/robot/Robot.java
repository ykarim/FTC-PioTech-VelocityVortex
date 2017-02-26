package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.sensors.gyro.AdafruitIMU;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.ArrayList;

/**
 * TODO: Use getters for hardware and ensure that if not present, app won't crash
 */
public class Robot {

    private HardwareMap hwMap = null;

    public DcMotor fl = null; //Front - Left Motor (Config == "FL")
    public DcMotor fr = null; //Front - Right Motor (Config == "FR")
    public DcMotor bl = null; //Back - Left Motor (Config == "BL")
    public DcMotor br = null; //Back - Right Motor (Config == "BR")

    public DcMotor intake = null; //Intake Motor (Config == "INTAKE")
    public DcMotor shoot = null; //Shoot Motor (Config == "SHOOT")

    public DcMotor cap = null; //Cap Motor (Config == "CAP")
    public Servo capServo = null;

    public ArrayList<DcMotor> driveMotors = new ArrayList<>(); //stores all drive motors
    public ArrayList<DcMotor> ballMotors = new ArrayList<>(); //stores all ball motors (intake, shoot)

    public OpticalDistanceSensor lightSensor = null; //Distance Sensor (Config == "OPTICAL")
    public VoltageSensor voltageSensor = null; //Voltage Sensor (Config == "Motor Controller 1")
    public UltrasonicSensor ultrasonicSensorLeft = null; //Ultrasonic Sensor (Config == "ULTRALEFT")
    public UltrasonicSensor ultrasonicSensorRight = null; //Ultrasoonic Sensor (Config =="ULTRARIGHT")

    public AdafruitIMU imu = null;


    /**
     * Initializes all drive and ball motors in NO ENCODERS mode
     * @param hwMap
     */
    public void initTeleOp(HardwareMap hwMap) {
        this.hwMap = hwMap;
        initDrive();
        initBall();
        initCap();
        initSensors();

        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            driveMotor.setPower(0);
        }
        for(DcMotor ballMotor : ballMotors) {
            ballMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            ballMotor.setPower(0);
        }
//        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        cap.setDirection(DcMotorSimple.Direction.FORWARD);
//        cap.setPower(0);
    }

    /**
     * Initializes all drive and ball motors in USING ENCODERS mode
     * @param hwMap
     */
    public void initAutoOp(LinearOpMode opMode, HardwareMap hwMap) {
        this.hwMap = hwMap;

        initDrive();
        initBall();
//        initCap();
        initSensors();

        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            opMode.idle();
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            driveMotor.setPower(0);
        }
        for(DcMotor ballMotor : ballMotors) {
            ballMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            opMode.idle();
            ballMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            ballMotor.setPower(0);
        }
//        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        cap.setDirection(DcMotorSimple.Direction.FORWARD);
//        cap.setPower(0);
    }

    /**
     * Initializes all drive and ball motors in USING ENCODERS mode
     * @param hwMap
     */
    public void initAutoOp(LinearVisionOpMode opMode, HardwareMap hwMap) {
        this.hwMap = hwMap;

        initDrive();
        initBall();
//        initCap();
        initSensors();

        for(DcMotor driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            driveMotor.setPower(0);
        }
        for(DcMotor ballMotor : ballMotors) {
            ballMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ballMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ballMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            ballMotor.setPower(0);
        }
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        cap.setDirection(DcMotorSimple.Direction.FORWARD);
//        cap.setPower(0);
    }

    private void initDrive() {
        fl = hwMap.dcMotor.get(RobotConstants.frontLeftMotor);
        fr = hwMap.dcMotor.get(RobotConstants.frontRightMotor);
        bl = hwMap.dcMotor.get(RobotConstants.backLeftMotor);
        br = hwMap.dcMotor.get(RobotConstants.backRightMotor);

        driveMotors.add(fl);
        driveMotors.add(fr);
        driveMotors.add(bl);
        driveMotors.add(br);
    }

    private void initBall() {
        intake = hwMap.dcMotor.get(RobotConstants.intakeMotor);
        shoot = hwMap.dcMotor.get(RobotConstants.shootMotor);

        ballMotors.add(intake);
        ballMotors.add(shoot);
    }

    private void initCap() {
        cap = hwMap.dcMotor.get(RobotConstants.capMotor);

        cap.setDirection(DcMotorSimple.Direction.FORWARD);
        cap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cap.setPower(0);

        capServo = hwMap.servo.get(RobotConstants.capServo);
        capServo.setPosition(0.9);
    }

    private void initSensors() {
        lightSensor = hwMap.opticalDistanceSensor.get(RobotConstants.opticalSensor);
        lightSensor.enableLed(true);

        voltageSensor = hwMap.voltageSensor.get(RobotConstants.voltageSensor);

        ultrasonicSensorLeft = hwMap.ultrasonicSensor.get(RobotConstants.ultrasonicSensorLeft);
        ultrasonicSensorRight = hwMap.ultrasonicSensor.get(RobotConstants.ultrasonicSensorRight);

        imu = new AdafruitIMU("IMU", hwMap);
        RobotConstants.homeHeadingAngle = imu.getHeading();
    }

    public void setDriveMotorMode(DcMotor.RunMode mode) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(mode);
        }
    }

    public void setDriveMotorPower(double power) {
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
    }
}
