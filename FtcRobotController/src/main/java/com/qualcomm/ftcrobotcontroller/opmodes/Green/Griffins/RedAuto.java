package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/12/2015.
 * The autonomous for the red alliance.
 */
public class RedAuto extends LinearOpMode{

    RobotHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(hardwareMap);

        waitForStart();

        //drive forward
        hardware.getLeftDriveMotor().setPower(-1);
        hardware.getRightDriveMotor().setPower(-1);
        sleep(400);

        //curve around to face ramp
        hardware.getRightDriveMotor().setPower(0);

        sleep(1500);

        //stop to extend arm
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //extend arm
        hardware.getArmTelescopeMotors().setPower(.25);
        while (hardware.getArmTelescopeMotors().getCurrentPosition() > -2500)
            waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();

        //drive forward onto ramp
        hardware.getLeftDriveMotor().setPower(-1);
        hardware.getRightDriveMotor().setPower(-1);

        sleep(5000);

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //extend arm
        hardware.getArmTelescopeMotors().setPower(-.25);
        while (hardware.getArmTelescopeMotors().getCurrentPosition() < -2000)
            waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();
    }
}