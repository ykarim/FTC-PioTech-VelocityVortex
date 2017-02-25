package org.firstinspires.ftc.teamcode.sensors.gyro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

@TeleOp (name= "Ada Test", group = "teletest")
public class AdaTest extends LinearVisionOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AdafruitIMU imu = new AdafruitIMU("IMU", this);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.x) {
                while (gamepad1.x) {}
                telemetry.addData("Heading", imu.getHeading());

                telemetry.addData("Pitch", imu.getPitch());

                telemetry.addData("Roll", imu.getRoll());
                telemetry.update();
            }

        }
    }
}
