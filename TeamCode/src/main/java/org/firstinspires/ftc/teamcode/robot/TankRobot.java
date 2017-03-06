package org.firstinspires.ftc.teamcode.robot;

public class TankRobot extends RobotInterface {

    public enum Directions implements DriveTrainDirections{

        NORTH(1.0, 1.0, -1.0, -1.0), SOUTH(-1.0, -1.0, 1.0, 1.0),
        ROTATE_RIGHT(1.0, 1.0, 1.0, 1.0), ROTATE_LEFT(-1.0, -1.0, -1.0, -1.0);

        private final double rearLeftPower, rearRightPower, frontLeftPower, frontRightPower;

        @Override
        public double getRearLeftPower() {
            return rearLeftPower;
        }

        @Override
        public double getRearRightPower() {
            return rearRightPower;
        }

        @Override
        public double getFrontLeftPower() {
            return frontLeftPower;
        }

        @Override
        public double getFrontRightPower() {
            return frontRightPower;
        }

        Directions(double rearLeftPower, double rearRightPower, double frontLeftPower, double frontRightPower) {
            this.rearLeftPower = rearLeftPower;
            this.rearRightPower = rearRightPower;
            this.frontLeftPower = frontLeftPower;
            this.frontRightPower = frontRightPower;
        }
    }



}
