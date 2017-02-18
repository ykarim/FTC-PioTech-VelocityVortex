package org.firstinspires.ftc.teamcode.OpModes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.RobotConstants;
import org.firstinspires.ftc.teamcode.Sensors.gyro.PhoneGyro;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;

/**
 * Results after testing:
 *    Inaccurate
 *    Too complex (requires constant calibration)
 *    Values must be linearized
 */
@Disabled
@Deprecated
@TeleOp (name = "Phone Gyro Test", group = "teletest")
public class PhoneGyroTest extends LinearOpMode{

    private String TAG = RobotConstants.teleOpTag + "Phone Gyro Test : ";
    PhoneGyro gyroSensor = new PhoneGyro();

    @Override
    public void runOpMode() {
        telemetry.addData(TAG, "Status : INITIALIZING");

        gyroSensor.start(hardwareMap);

        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);

        telemetry.addData(TAG, "Status : READY");
        waitForStart();

        while (opModeIsActive()) {
            updateTelemetryData();
            OpModeUtils.waitFor(this, 5);
        }
    }

    private void updateTelemetryData() {
        telemetry.addData(TAG, "Status : RUNNING");

        telemetry.addData(TAG, "Gyro X : " + gyroSensor.getValues()[0]);
        telemetry.addData(TAG, "Gyro Y : " + gyroSensor.getValues()[1]);
        telemetry.addData(TAG, "Gyro Z : " + gyroSensor.getValues()[2]);
        telemetry.addData(TAG, "Gyro Time : " + gyroSensor.getValues()[3]);

        telemetry.update();
    }
}
