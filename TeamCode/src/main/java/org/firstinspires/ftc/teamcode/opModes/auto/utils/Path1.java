package org.firstinspires.ftc.teamcode.opModes.auto.utils;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotMovement;
import org.firstinspires.ftc.teamcode.robot.RobotUtilities;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public class Path1 {

    private Robot robot = null;
    private RobotMovement robotMovement = new RobotMovement(robot);
    private RobotUtilities robotUtilities = new RobotUtilities(robot);

    private int posVector[] = new int[2];  //Stores position to dynamically change path

    public Path1(Robot robot) {
        this.robot = robot;
    }

    public void goForBeaconOne(LinearVisionOpMode opMode, Beacon.BeaconAnalysis analysis,
                                    Robot.TeamColor teamColor) {
        if (posVector[0] == 84 && posVector[1] == 12) {
            robotMovement.move(RobotMovement.Direction.NORTH, 18);

            robotMovement.move(RobotMovement.Direction.EAST, 24);
            robotMovement.move(RobotMovement.Direction.NORTH, 36);

            robotMovement.move(RobotMovement.Direction.EAST, 28);

            robotUtilities.alignWithLine(opMode, RobotMovement.Direction.EAST, 5);
            robotUtilities.pushBeaconButton(analysis, teamColor);

            posVector[0] = posVector[0] + 18 + 36;
            posVector[1] = posVector[1] + 24 + 28;
        }
    }

    public void goForBeaconTwo(LinearVisionOpMode opMode, Beacon.BeaconAnalysis analysis,
                                      Robot.TeamColor teamColor) {
        if (posVector[0] == 138 && posVector[1] == 64) {
            robotUtilities.alignWithLine(opMode, RobotMovement.Direction.WEST, 7);
            robotUtilities.pushBeaconButton(analysis, teamColor);
        }
    }

    public void goToShoot(LinearVisionOpMode opMode) {
        robotMovement.move(RobotMovement.Direction.NORTH, 12);
        posVector[0] = 84;
        posVector[1] = 12;
        robotUtilities.shootDoubleBall(opMode, 10);
    }

    /**
     * Ending move
     * @param teamColor
     */
    public void goForVortex(Robot.TeamColor teamColor) {
        if (teamColor == Robot.TeamColor.BLUE) {
            robotMovement.move(RobotMovement.Direction.WEST, 6);
        }
        robotMovement.move(RobotMovement.Direction.SOUTH, 96);
    }

    /**
     * Ending move
     */
    public void goForCapBall() {

    }

    public void goForCenterPark() {

    }

}
