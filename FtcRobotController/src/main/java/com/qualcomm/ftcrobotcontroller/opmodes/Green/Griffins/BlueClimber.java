package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by David on 2/5/2016.
 */
public class BlueClimber extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hardware = new RobotHardware(hardwareMap);
        double driveSpeed = .5;
        hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();
/*
        //extend arm
        hardware.getArmTelescopeMotors().setPower(-.25);
        while (hardware.getArmTelescopeMotors().getCurrentPosition() < -2000)
            waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();*/

        int firstOut = -3140;
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
        hardware.getRightDriveMotor().setPower(0);
    }
}
