package org.firstinspires.ftc.teamcode.opModes.auto.utils;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

public abstract class Path {

    public abstract void goForBeaconOne(LinearVisionOpMode opMode, Beacon.BeaconAnalysis analysis,
                                        Robot.TeamColor teamColor);

}
