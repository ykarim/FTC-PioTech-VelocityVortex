package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.utils.Color;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Disabled
public class BaseAutoOp extends LinearVisionOpMode {

    public String TAG = "Auto : ";

    @Override
    public void runOpMode() throws InterruptedException {}

    public final void getData() {
        //TODO: Should fill variables with data from SharedPreferences
    }

    public final void delay() {
        double secDelay;

        try {
            secDelay = 0;
        } catch (NumberFormatException nfe) {
            secDelay = 0;
        }

        wait(secDelay);
    }

    public final Color getTeamColor() {
        boolean blueChecked = FtcRobotControllerActivity.blueTeamColor.isChecked();
        boolean redChecked = FtcRobotControllerActivity.redTeamColor.isChecked();

        if(blueChecked) {
            return Color.BLUE;
        } else if(redChecked) {
            return Color.RED;
        }
        return Color.NA;
    }

    public void initVision() {
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

    public final void write(String... text) {
        for (String msg : text) {
            telemetry.addData(TAG, msg);
        }
        telemetry.update();
    }

    public final void setTag(String opName) {
        TAG += opName;
    }

    public void wait(double sec) {
        if (sec != 0) {
            long millis = Math.round(sec * 1000);
            long stopTime = System.currentTimeMillis() + millis;
            while (opModeIsActive() && System.currentTimeMillis() < stopTime) {
                try {
                    waitOneFullHardwareCycle();
                } catch (Exception ex) {}
            }
        }
    }
}
