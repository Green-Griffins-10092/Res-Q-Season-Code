package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 2/14/2016.
 * The master autonomous for the ramp.
 */
public abstract class SimpleRampAuto extends LinearOpMode {

    protected static boolean blueSide = true;
    protected static int wait = 0;

    RobotHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Instructions 1", "Retract arm using teleop, after arm is in,\nrun the spool in reverse until there is a small amount of tension on the cable.");
        telemetry.addData("Instructions 2", "Restart robot, using the button on the driver station, to reset encoders.");
        telemetry.addData("Instructions 3", "Set up on field, on the outer edge of the tile next to the ramp.  Press init and set auto timer.\nWait for signal to start auto to press start.");

        hardware = new RobotHardware(hardwareMap);

        hardware.getRobotRotationGyro().calibrate();

        hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        sleep(wait);

        while (hardware.getRobotRotationGyro().isCalibrating()) {
            waitForNextHardwareCycle();
        }

        //drive forward
        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);
        sleep(500);
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        int gyroTarget = 130;
        DcMotor turningMotor;
        //curve around to face ramp
        if (blueSide) {
            gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + gyroTarget;
            turningMotor = hardware.getLeftDriveMotor();
        } else {
            gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget;
            turningMotor = hardware.getRightDriveMotor();
        }

        ElapsedTime timeout = new ElapsedTime();
        do {
            waitForNextHardwareCycle();
            int headingError = (blueSide?1:-1) * (gyroTarget - hardware.getRobotRotationGyro().getIntegratedZValue());
            double drivePower = headingError / 130.0;
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < .25 && drivePower != 0) {
                drivePower = (.25 * drivePower / Math.abs(drivePower));
            }
            turningMotor.setPower(drivePower);
            telemetry.addData("error", headingError);
            telemetry.addData("Motor power", drivePower);
        } while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue()-gyroTarget) > 5 && timeout.time() < 5);

        //stop to extend arm
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        // TODO: 2/14/2016 Raise churro grabber here

        //drive forward onto ramp
        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);

        sleep(3000);

        // TODO: 2/14/2016 Lower churro grabber here

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //extend arm
        hardware.getArmTelescopeMotors().setPower(.25);
        timeout.reset();
        while (hardware.getArmTelescopeMotors().getCurrentPosition() > -2500 && timeout.time() < 2)
            waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();
    }
}