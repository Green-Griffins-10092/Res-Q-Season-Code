package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by David on 2/5/2016.
 */

public abstract class ClimberAuto extends LinearOpMode {

    protected static boolean blueSide = true;
    protected static int waitInMilliseconds = 0;
    RobotHardware hardware;

    public void twoWheelTurn(double angle) throws InterruptedException {

        //curve around to face the ramp
        int gyroTarget;
        gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double minimumPower = .15;
        double drivePower = 1.0;
        do {
            waitForNextHardwareCycle();
            int headingError = (gyroTarget - hardware.getRobotRotationGyro().getIntegratedZValue());
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            hardware.getLeftDriveMotor().setPower(drivePower);
            hardware.getRightDriveMotor().setPower(-drivePower);
            drivePower = headingError / (2 * Math.abs(angle));
            drivePower = Range.clip(drivePower, -1, 1);
            RobotLog.i("2w -----------------------");
            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            telemetry.addData("error", headingError);
            telemetry.addData("target, current", gyroTarget + ", " + hardware.getRobotRotationGyro().getIntegratedZValue());
            telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget) > 0 && timeout.time() < 5);

        //stop motors
        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
    }

    public void driveStraight(long encoderCount, DriveStraightDirection direction, double power) throws InterruptedException {
        if (power < 0) {
            throw new IllegalArgumentException("Power must be greater than 0");
        } else if (power > 1) {
            throw new IllegalArgumentException("Power must be less than 1");
        }
        if (encoderCount < 0) {
            throw new IllegalArgumentException(" Encoder count must be greater than 0");
        }
        ElapsedTime timeout = new ElapsedTime();
        //encoder target
        long encoderTarget;

        if (direction == DriveStraightDirection.BACKWARD) {
            power = -power;
            encoderTarget = hardware.getLeftDriveMotor().getCurrentPosition() - encoderCount;
        } else {
            encoderTarget = hardware.getLeftDriveMotor().getCurrentPosition() + encoderCount;
        }
        //drive forward, clearing front of ramp
        timeout.reset();
        boolean stopCondition;
        do {
            waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(power);
            hardware.getRightDriveMotor().setPower(power);

            if (direction == DriveStraightDirection.BACKWARD) {
                stopCondition = hardware.getLeftDriveMotor().getCurrentPosition() > encoderTarget;
            } else {
                stopCondition = hardware.getLeftDriveMotor().getCurrentPosition() < encoderTarget;
            }
        }
        while (stopCondition && timeout.time() < 5);

        //stop motors
        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();

    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(hardwareMap);
        ElapsedTime timeout = new ElapsedTime();

        hardware.getRobotRotationGyro().calibrate();

        hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        sleep(waitInMilliseconds);

        while (hardware.getRobotRotationGyro().isCalibrating()) {
            waitForNextHardwareCycle();
        }

//        //extend arm
//        hardware.getArmTelescopeMotors().setPower(.25);
//        timeout.reset();
//        while (hardware.getArmTelescopeMotors().getCurrentPosition() > -2500 && timeout.time() < 2)
//            waitForNextHardwareCycle();
//        hardware.getArmTelescopeMotors().setPowerFloat();
//        waitForNextHardwareCycle();

        sleep(1000);

        waitForNextHardwareCycle();

        //encoder target
        int encoderTarget = 9000 + hardware.getLeftDriveMotor().getCurrentPosition();

        //drive forward, clearing front of ramp
        timeout.reset();
        do {
            waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(.5);
            hardware.getRightDriveMotor().setPower(.5);
        }
        while (hardware.getLeftDriveMotor().getCurrentPosition() < encoderTarget && timeout.time() < 5);

        //stop motors
        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        int angle = 39;
        if (blueSide) {
            twoWheelTurn(angle);
        } else {
            twoWheelTurn(-angle);
        }

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        driveStraight(1700, DriveStraightDirection.FORWARD, .5);

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        driveStraight(1700, DriveStraightDirection.BACKWARD, .5);

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        /*int firstOut = -3140;
        hardware.getLeftDriveMotor().setTargetPosition(firstOut);
        hardware.getRightDriveMotor().setTargetPosition(firstOut);
        hardware.getLeftDriveMotor().setPower(-driveSpeed);
        hardware.getRightDriveMotor().setPower(-driveSpeed);

        while (hardware.getLeftDriveMotor().getCurrentPosition() > firstOut && hardware.getRightDriveMotor().getCurrentPosition() > firstOut);
        {
            waitForNextHardwareCycle();
        }

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        int firstTurnLeft = -2277;
        int firstTurnRight = -3595;

        hardware.getLeftDriveMotor().setTargetPosition(firstTurnLeft);
        hardware.getRightDriveMotor().setTargetPosition(firstTurnRight);
        hardware.getLeftDriveMotor().setPower(driveSpeed);
        hardware.getRightDriveMotor().setPower(-driveSpeed);

        while (hardware.getLeftDriveMotor().getCurrentPosition() < firstTurnLeft && hardware.getRightDriveMotor().getCurrentPosition() > firstTurnRight);
        {
            waitForNextHardwareCycle();
        }

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        int secondDriveLeft = -7961;
        int secondDriveRight = -9279;

        hardware.getLeftDriveMotor().setTargetPosition(secondDriveLeft);
        hardware.getRightDriveMotor().setTargetPosition(secondDriveRight);
        hardware.getLeftDriveMotor().setPower(-driveSpeed);
        hardware.getRightDriveMotor().setPower(-driveSpeed);

        while (hardware.getLeftDriveMotor().getCurrentPosition() > secondDriveLeft && hardware.getRightDriveMotor().getCurrentPosition() > secondDriveRight);
        {
            waitForNextHardwareCycle();
        }

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);*/
    }

    enum DriveStraightDirection {FORWARD, BACKWARD}
}
