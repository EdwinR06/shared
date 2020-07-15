package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Chassis {
    private DriveWheel frontLeft;
    private DriveWheel frontRight;
    private DriveWheel backLeft;
    private DriveWheel backRight;

    public Chassis() {
        FTCUtil.telemetry.addData("Status", "Initialized");
        frontLeft = new DriveWheel("front_left_motor", DcMotor.Direction.FORWARD);
        frontRight = new DriveWheel("front_right_motor", DcMotor.Direction.FORWARD);
        backLeft = new DriveWheel( "back_left_motor", DcMotor.Direction.REVERSE);
        backRight = new DriveWheel( "back_right_motor", DcMotor.Direction.REVERSE);
    }

    public void drive(double inches, double motorPower) {
        setTargetDistance(inches);
        setPowers(motorPower);
    }

    private void setTargetDistance(double inches){
        frontLeft.setTargetDistance(inches);
        frontRight.setTargetDistance(inches);
        backLeft.setTargetDistance(inches);
        backRight.setTargetDistance(inches);
    }

    private void setPowers(double motorPower){
        frontLeft.setPower(motorPower);
        frontRight.setPower(motorPower);
        backLeft.setPower(motorPower);
        backRight.setPower(motorPower);
    }

    /*public void strafe(double distance) {
        double strafe;
        if (distance > 0) {
            strafe = 1;
        } else {
            strafe = -1;
        }
        double turn = 0;
        double drive = 0;

        while () {
            frontLeft.setPower(drive + strafe + turn);
            backLeft.setPower(drive - strafe + turn);
            frontRight.setPower(drive - strafe - turn);
            backRight.setPower(drive + strafe - turn);
        }
    }

    public void turn(double distance) {
        double drive = 0;
        double strafe = 0;
        double turn = 1;
        while () {
            frontLeft.setPower(drive + strafe + turn);
            backLeft.setPower(drive - strafe + turn);
            frontRight.setPower(drive - strafe - turn);
            backRight.setPower(drive + strafe - turn);
        }
    }*/

    public void followPath() {
        frontLeft.setPower(.5);
        backLeft.setPower(.5);
        frontRight.setPower(.5);
        backRight.setPower(.5);
    }
}
