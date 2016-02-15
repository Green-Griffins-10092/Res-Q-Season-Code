package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by David on 2/14/2016.
 * The master autonomous for the ramp.
 */
public abstract class RampAuto extends LinearOpMode {

    protected static boolean blueSide = true;

    RobotHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Instructions 1", "Retract arm using teleop, after arm is in,\nrun the spool in reverse until there is a small amount of tension on the cable.");
        telemetry.addData("Instructions 2", "Restart robot, using the button on the driver station, to reset encoders.");
        telemetry.addData("Instructions 3", "Set up on field, on the outer edge of the tile next to the ramp.  Press init and set auto timer.\nWait for signal to start auto to press start.");

        hardware = new RobotHardware(hardwareMap);

        hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        //drive forward
        hardware.getLeftDriveMotor().setPower(-1);
        hardware.getRightDriveMotor().setPower(-1);
        sleep(400);

        //curve around to face ramp
        if (blueSide) {
            hardware.getLeftDriveMotor().setPower(0);
        } else {
            hardware.getRightDriveMotor().setPower(0);
        }

        sleep(1600);

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