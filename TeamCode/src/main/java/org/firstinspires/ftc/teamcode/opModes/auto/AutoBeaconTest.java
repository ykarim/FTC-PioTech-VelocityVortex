package org.firstinspires.ftc.teamcode.opModes.auto;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Autonomous(name = "Double Beacon Test", group = "AutoTest")
public class AutoBeaconTest extends LinearVisionOpMode{

    private Robot robot = new Robot();
    private RobotMovement robotMovement = new RobotMovement();
    private RobotUtilities robotUtilities = new RobotUtilities();

    private final int RED_ALLIANCE = 1;
    private final int BLUE_ALLIANCE = 2;

    @Override
    public void runOpMode() throws InterruptedException{
        waitForVisionStart();
        initVision();

        robot.initAutoOp(this, hardwareMap);
        robotMovement.init(robot);
        robotUtilities.init(robot);

        int teamColor = getTeamColor();
//        int teamColor = BLUE_ALLIANCE;
        boolean blueLeft, blueRight, redLeft, redRight;

        waitForStart();

        while (opModeIsActive()) {
            robotMovement.move(RobotMovement.Direction.NORTH, 36);
            robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT, 90);
            robotMovement.move(RobotMovement.Direction.NORTH, 48);
            robotUtilities.alignWithLine(RobotMovement.Direction.EAST);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            blueRight = beacon.getAnalysis().isRightBlue();
            redLeft = beacon.getAnalysis().isLeftRed();
            redRight = beacon.getAnalysis().isRightRed();

            if (teamColor == BLUE_ALLIANCE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon One on Right");
                }
            } else if (teamColor == RED_ALLIANCE) {
                if (redLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Left");
                } else if (redRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Red Beacon One on Right");
                }
            }

            robotUtilities.alignWithLine(RobotMovement.Direction.WEST);

            blueLeft = beacon.getAnalysis().isLeftBlue();
            redLeft = beacon.getAnalysis().isLeftRed();

            if (teamColor == BLUE_ALLIANCE) {
                if (blueLeft) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Left");
                } else if (blueRight) {
                    telemetry.addData(RobotConstants.autoOpTag, "Hit Blue Beacon Two on Right");
                }
            } else if (teamColor == RED_ALLIANCE) {
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
     * Reads Team Color from SharedPreferences
     * @return int representing team color (RED_ALLIANCE or BLUE_ALLIANCE)
     */
    private int getTeamColor() {
        Activity activity = (Activity) hardwareMap.appContext;
        SharedPreferences preferences = activity.getSharedPreferences("Color", Context.MODE_PRIVATE);

        if(preferences.getString("Color", "") == "BLUE") {
            return BLUE_ALLIANCE;
        } else if(preferences.getString("Color", "") == "RED") {
            return RED_ALLIANCE;
        }
        return 0;
    }
}
