package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class OpModeUtils {

    /**
     * Reads Team Color from FtcRobotControllerActivity
     * @return Robot.TeamColor
     */
    public static Robot.TeamColor getTeamColor() {
        boolean blueChecked = FtcRobotControllerActivity.blueTeamColor.isChecked();
        boolean redChecked = FtcRobotControllerActivity.redTeamColor.isChecked();

        if(blueChecked) {
            return Robot.TeamColor.BLUE;
        } else if(redChecked) {
            return Robot.TeamColor.RED;
        }
        return Robot.TeamColor.NONE;
    }

    /**
     * Returns int of sec to wait before running program
     * @return
     */
    public static int getDelay() {
        try {
            return Integer.parseInt(FtcRobotControllerActivity.autoDelay.getText().toString());
        } catch (NumberFormatException nfe) {
            return 0;
        }
    }

    public static void addToTelemetry (OpMode opMode, String TAG, String... msg) {
        for (String text : msg) {
            opMode.telemetry.addData(TAG, text);
        }
        opMode.telemetry.update();
    }

    public static void addToTelemetry (LinearOpMode opMode, String TAG, String... msg) {
        for (String text : msg) {
            opMode.telemetry.addData(TAG, text);
        }
        opMode.telemetry.update();
    }

    public static void addToTelemetry (LinearVisionOpMode opMode, String TAG, String... msg) {
        for (String text : msg) {
            opMode.telemetry.addData(TAG, text);
        }
        opMode.telemetry.update();
    }

    /**
     * Wait a period of time. This will be non-blocking, so Thread away!
     * @param sec time to wait in seconds.
     */
    public static void waitFor(LinearOpMode opMode, int sec) {
        long millis = sec * 1000;
        long stopTime = System.currentTimeMillis() + millis;
        while(opMode.opModeIsActive() && System.currentTimeMillis() < stopTime) {
            try {
                opMode.waitOneFullHardwareCycle();
            } catch(Exception ex) {}
        }
    }

    /**
     * Wait a period of time. This will be non-blocking, so Thread away!
     * @param sec time to wait in seconds.
     */
    public static void waitFor(LinearVisionOpMode opMode, int sec) {
        if (sec != 0) {
            long millis = sec * 1000;
            long stopTime = System.currentTimeMillis() + millis;
            while (opMode.opModeIsActive() && System.currentTimeMillis() < stopTime) {
                try {
                    opMode.waitOneFullHardwareCycle();
                } catch (Exception ex) {
                }
            }
        }
    }
}
