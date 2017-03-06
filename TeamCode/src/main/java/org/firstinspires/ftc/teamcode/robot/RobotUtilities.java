package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sensors.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.sensors.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Color;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import static java.lang.Thread.sleep;

public class RobotUtilities {

    private Robot robot = null;

    public boolean continuousIntake = false;
    public boolean continuousShoot = false;

    private boolean lightLED = true;

    private boolean secondUltraTest = false;

    private double prevPos= 0;
    private double previousTime = 0;

    public RobotUtilities(Robot robot) {
        this.robot = robot;
    }

    public void capBall() {
        //TODO
    }

    /**
     * Pushes beacon button once aligned with wall
     * @param robot
     */
    public void pushBeacon(Robot robot, Color desiredColor) {
        RobotMovement robotMovement = new RobotMovement(robot);
        robotMovement.orient(RobotMovement.Orientation.RIGHT);

        if (BeaconStatus.getLeftColor() != desiredColor &&
                BeaconStatus.getRightColor() != desiredColor) {
            robotMovement.move(RobotMovement.Direction.NORTH, 4);
            robotMovement.move(RobotMovement.Direction.SOUTH, 2);
        }
    }

    public void toggleLightLED() {
        if (lightLED) {
            lightLED = false;
            robot.lightSensor.enableLed(lightLED);
        } else {
            lightLED = true;
            robot.lightSensor.enableLed(lightLED);
        }
    }

    public void shootDoubleBall(LinearOpMode opMode, double timeout) {
        long stop = System.currentTimeMillis() + Math.round(timeout * 1000);

        shootBalls(true);
        OpModeUtils.waitFor(opMode, RobotConstants.shotWaitPeriod);

        if (System.currentTimeMillis() < stop) {
            intakeBalls(true);
            OpModeUtils.waitFor(opMode, RobotConstants.intakeWaitPeriod);
        }

        intakeBalls(false);
        shootBalls(false);
    }

    public void shootDoubleBall(LinearVisionOpMode opMode, double timeout) {
        long stop = System.currentTimeMillis() + Math.round(timeout * 1000);

        shootBalls(true);
        OpModeUtils.waitFor(opMode, RobotConstants.shotWaitPeriod);

        if (System.currentTimeMillis() < stop) {
            intakeBalls(true);
            OpModeUtils.waitFor(opMode, RobotConstants.intakeWaitPeriod);
        }

        intakeBalls(false);
        shootBalls(false);
    }

    public void intakeBalls(boolean condition) {
        if (condition) {
            robot.intake.setPower(-RobotConstants.intakeSpeed);
        } else {
            robot.intake.setPower(0);
        }
    }

    public void shootBalls(boolean condition) {
        if (condition) {
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        } else {
            robot.shoot.setPower(0);
        }
    }

    public void continuousIntake() {
        if (!continuousIntake) {
            continuousIntake = true;
            robot.intake.setPower(-RobotConstants.intakeSpeed);
        } else {
            continuousIntake = false;
            robot.intake.setPower(0);
        }
    }

    public void continuousShoot() {
        if (!continuousShoot) {
            continuousShoot = true;
            robot.shoot.setPower(-RobotConstants.shootSpeed);
        } else {
            continuousShoot = false;
            robot.shoot.setPower(0);
        }
    }

    /**
     * Aligns ODS by moving left or right given which side line is located on
     * @param direction
     */
    public void alignWithLine(RobotMovement.Direction direction, int timeoutSec) {
        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (!lightLED) {
            toggleLightLED();
        }

        RobotMovement robotMovement = new RobotMovement(robot);
        RobotConstants.moveSpeed = 0.4;
        robotMovement.orient(RobotMovement.Orientation.RIGHT); //Sensors are currently on the right
        robotMovement.move(direction);

        while (robot.lightSensor.getLightDetected() < RobotConstants.perfectWhiteLineValue
                && time.seconds() < timeoutSec) {
            if (robot.lightSensor.getLightDetected() > RobotConstants.whiteLineValue) {
                RobotConstants.moveSpeed = 0.0;
                break;
            } else if (robot.lightSensor.getLightDetected() > 0.3) {
                RobotConstants.moveSpeed = 0.2;
            } else if (robot.lightSensor.getLightDetected() > 0.2) {
                RobotConstants.moveSpeed = 0.3;
            } else {
                RobotConstants.moveSpeed = 0.4;
            }
            robotMovement.move(direction);
        }
        robotMovement.move(RobotMovement.Direction.NONE);

        RobotConstants.moveSpeed = 1.0;
        toggleLightLED();

        // Add PID loop to slow down once reaching
    }

    //TODO: Change for Blue or Red
    public boolean alignWithWallUsingGyro() {
        RobotMovement robotMovement = new RobotMovement(robot);

        if (robot.imu.getHeading() < RobotConstants.perfectGyroAngleMin) {
            while (robot.imu.getHeading() < RobotConstants.perfectGyroAngleMin) {
                robotMovement.rotate(RobotMovement.Direction.ROTATE_LEFT,
                        Math.abs(robot.imu.getHeading() - RobotConstants.perfectGyroAngleMin));
            }
        } else if (robot.imu.getHeading() > RobotConstants.perfectGyroAngleMax) {
            while (robot.imu.getHeading() > RobotConstants.perfectGyroAngleMax) {
                robotMovement.rotate(RobotMovement.Direction.ROTATE_RIGHT,
                        Math.abs(robot.imu.getHeading() - RobotConstants.perfectGyroAngleMax));
            }
        }
        return false;
    }

    public double getGyroAngle() {
        return (robot.imu.getHeading() - RobotConstants.homeHeadingAngle);
    }


    public double getShooterRate() {
        double posC = robot.shoot.getCurrentPosition() - prevPos;
        try {
            sleep(100);
        } catch (InterruptedException ie) {

        }
        double tChange = System.nanoTime() - previousTime;
        previousTime = System.nanoTime();
        tChange = tChange / 1e9;
        prevPos = robot.shoot.getCurrentPosition();
        return posC / tChange;
    }
}
