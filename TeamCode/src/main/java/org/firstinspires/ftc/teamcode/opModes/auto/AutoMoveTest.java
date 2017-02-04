package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;

@Autonomous (name = "Distance Test", group = "AutoTest")
public class AutoMoveTest extends LinearOpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement();

    public void runOpMode() {
        robot.initAutoOp(this, hardwareMap);
        robotMovement.init(robot);

        telemetry.addData(RobotConstants.autoOpTag, "Start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData(RobotConstants.autoOpTag, "Moving Forward 20 in.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.move(this, RobotMovement.Direction.NORTH, 20.0);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            telemetry.addData(RobotConstants.autoOpTag, "Moving Backward 20 in.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.move(this, RobotMovement.Direction.SOUTH, 20.0);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            telemetry.addData(RobotConstants.autoOpTag, "Moving Right 20 in.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.move(this, RobotMovement.Direction.EAST, 20.0);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            telemetry.addData(RobotConstants.autoOpTag, "Moving Left 20 in.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.move(this, RobotMovement.Direction.WEST, 20.0);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            telemetry.addData(RobotConstants.autoOpTag, "Rotate Right 90 deg.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.rotate(this, RobotMovement.Direction.ROTATE_RIGHT, 90);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            telemetry.addData(RobotConstants.autoOpTag, "Rotate Left 90 deg.");
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getCurrentPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getCurrentPosition());
            robotMovement.rotate(this, RobotMovement.Direction.ROTATE_LEFT, 90);
            telemetry.addData(RobotConstants.autoOpTag, robot.fl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.fr.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.bl.getTargetPosition());
            telemetry.addData(RobotConstants.autoOpTag, robot.br.getTargetPosition());
            telemetry.update();

            requestOpModeStop();
        }
    }
}
