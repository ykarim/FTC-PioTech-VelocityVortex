package org.firstinspires.ftc.teamcode.sensors.gyro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.lasarobotics.vision.opmode.LinearVisionOpMode;

@TeleOp (name= "Ada Test", group = "teletest")
public class AdaTest extends LinearVisionOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("WORKING", "");

            telemetry.addData("Heading", imu.getHeading());

            telemetry.addData("Pitch", imu.getPitch());

            telemetry.addData("Roll", imu.getRoll());
            telemetry.update();

            sleep(1000); //goes offline if queried too fast
        }
    }
}
