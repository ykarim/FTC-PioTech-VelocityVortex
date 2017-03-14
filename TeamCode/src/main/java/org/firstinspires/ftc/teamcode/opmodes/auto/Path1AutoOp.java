package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.TankRobot;

@Autonomous (name = "Path1", group = "auto")
public class Path1AutoOp extends BaseAutoOp {

    @Override
    public void runOpMode() throws InterruptedException {
        leo = new TankRobot(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            delay();
        }
    }
}