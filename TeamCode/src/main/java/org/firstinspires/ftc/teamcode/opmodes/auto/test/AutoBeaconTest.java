package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.firstinspires.ftc.teamcode.utils.Color;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Disabled //until path is changed
@Autonomous(name = "Double Beacon Test", group = "autotest")
public class AutoBeaconTest extends LinearVisionOpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);

    @Override
    public void runOpMode() throws InterruptedException{
        waitFor(OpModeUtils.getDelay());
        waitForVisionStart();
        initVision();

        robot.initAutoOp(this, hardwareMap);

        Color teamColor = OpModeUtils.getTeamColor();
        boolean blueLeft, blueRight, redLeft, redRight;

        waitForStart();

        while (opModeIsActive()) {
            robotMovement.move(RobotMovement.Direction.NORTH, 36);
            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
            robotMovement.move(RobotMovement.Direction.NORTH, 48);
            robotUtilities.alignWithLine(RobotMovement.Direction.EAST, 5);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            blueRight = beacon.getAnalysis().isRightBlue();
            redLeft = beacon.getAnalysis().isLeftRed();
            redRight = beacon.getAnalysis().isRightRed();

            if (teamColor == Color.BLUE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Right");
                }
            } else if (teamColor == Color.RED) {
                if (redLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Left");
                } else if (redRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Right");
                }
            }

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST, 5);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            redLeft = beacon.getAnalysis().isLeftRed();

            if (teamColor == Color.BLUE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Right");
                }
            } else if (teamColor == Color.RED) {
                if (redLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon Two on Left");
                } else if (redRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon Two on Right");
                }
            }

            telemetry.update();
            requestOpModeStop();
        }
    }

    private void initVision() {
        this.setCamera(Cameras.SECONDARY);
        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }

    /**
     * Wait a period of time. This will be non-blocking, so Thread away!
     * @param sec time to wait in seconds.
     */
    private void waitFor(int sec) {
        long millis = sec * 1000;
        long stopTime = System.currentTimeMillis() + millis;
        while(opModeIsActive() && System.currentTimeMillis() < stopTime) {
            try {
                waitOneFullHardwareCycle();
            } catch(Exception ex) {}
        }
    }
}