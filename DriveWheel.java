package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveWheel {
    private static final double ticksPerInch = 360;
    private DcMotor dcMotor;
    private String deviceName;

    public DriveWheel(String deviceName, DcMotor.Direction direction) {
        this.deviceName = deviceName;
        dcMotor = FTCUtil.hardwareMap.get(DcMotor.class, deviceName);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setDirection(direction);
    }

    public void setPower(double power){
        dcMotor.setPower(power);
    }
    public void setTargetDistance(double distanceInches){
        double motorTarget = dcMotor.getCurrentPosition() + (distanceInches * ticksPerInch);
        dcMotor.setTargetPosition((int) motorTarget);
    }
}
