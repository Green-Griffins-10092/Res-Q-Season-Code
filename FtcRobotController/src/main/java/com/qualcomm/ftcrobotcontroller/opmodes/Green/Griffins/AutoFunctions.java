package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by David on 2/22/2016.
 * For the auto functions
 */
public class AutoFunctions {
    private LinearOpMode linearOpMode;
    private RobotHardware hardware;

    public AutoFunctions(RobotHardware hardware, LinearOpMode linearOpMode) {
        this.hardware = hardware;
        this.linearOpMode = linearOpMode;
    }

    public void oneWheelTurn(DcMotor turningMotor, double angle) throws InterruptedException {
        double minimumPower = .2;
        int gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double drivePower = 1.0;
        do {
            linearOpMode.waitForNextHardwareCycle();
            int headingError = (gyroTarget - hardware.getRobotRotationGyro().getIntegratedZValue());

            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            RobotLog.i("-----------------------");
            turningMotor.setPower(drivePower);
            drivePower = headingError / (2 * angle);
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget) > 0 && timeout.time() < 5);

        //stop driving
        linearOpMode.waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        linearOpMode.waitForNextHardwareCycle();
    }

    public void twoWheelTurn(double angle) throws InterruptedException {

        //curve around to face the ramp
        int gyroTarget;
        gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double minimumPower = .10;
        double drivePower = 1.0;
        do {
            linearOpMode.waitForNextHardwareCycle();
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
            linearOpMode.telemetry.addData("error", headingError);
            linearOpMode.telemetry.addData("target, current", gyroTarget + ", " + hardware.getRobotRotationGyro().getIntegratedZValue());
            linearOpMode.telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget) > 0 && timeout.time() < 5);

        //stop motors
        linearOpMode.waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        linearOpMode.waitForNextHardwareCycle();
    }

    public void driveStraight(long encoderCount, DriveStraightDirection direction, double power) throws InterruptedException {
        double minimumPower = .1;
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
        RobotLog.i("Drive Straight --------------");
        RobotLog.i("Direction: " + direction);
        RobotLog.i("Power: " + power);
        RobotLog.i("Encoder Counts: " + encoderCount);

        long leftEncoderOffset = hardware.getLeftDriveMotor().getCurrentPosition();
        long rightEncoderOffset = hardware.getRightDriveMotor().getCurrentPosition();

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

            //how much the left motor is ahead of the right motor.
            long encoderDifference = (hardware.getLeftDriveMotor().getCurrentPosition() - leftEncoderOffset) - (hardware.getRightDriveMotor().getCurrentPosition() - rightEncoderOffset);
            long error = hardware.getLeftDriveMotor().getCurrentPosition() - encoderDifference / 2;

            //calculate using encoder difference
            double powerOffset = 0;

            power = encoderCount / (2 * error);

            if (Math.abs(power) < minimumPower) {
                power = minimumPower * Math.signum(power);
            }

            RobotLog.i("DriveStraight loop------");
            RobotLog.i("Encoder Counts to go: " + error);
            RobotLog.i("Power: " + power);
            RobotLog.i("Encoder difference: " + encoderDifference);
            RobotLog.i("Power offset: " + powerOffset);
            linearOpMode.telemetry.addData("Encoder Counts to go", Math.abs(hardware.getLeftDriveMotor().getCurrentPosition() - encoderTarget));

            linearOpMode.waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(power - powerOffset);
            hardware.getRightDriveMotor().setPower(power + powerOffset);

            stopCondition = Math.abs(hardware.getLeftDriveMotor().getCurrentPosition() - encoderTarget) < 3;
        } while (stopCondition && timeout.time() < 10);

        //stop motors
        linearOpMode.waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        linearOpMode.waitForNextHardwareCycle();
    }

    public void extendArm() throws InterruptedException {
        ElapsedTime timeout = new ElapsedTime();
        int target = hardware.getArmTelescopeMotors().getCurrentPosition() - 5000;
        hardware.getArmTelescopeMotors().setPower(.25);
        timeout.reset();
        while (hardware.getArmTelescopeMotors().getCurrentPosition() > target && timeout.time() < 2)
            linearOpMode.waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();
        linearOpMode.waitForNextHardwareCycle();
    }

    public enum DriveStraightDirection {FORWARD, BACKWARD}
}
