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
public abstract class RampAuto extends LinearOpMode {

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

        int gyroTarget = 40;
        DcMotor turningMotor;
        //curve around to be parallel to ramp
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
        } while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue()-gyroTarget) > 1 && timeout.time() < 5);

        //stop driving
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        //encoder target
        int encoderTarget = 4700 + hardware.getLeftDriveMotor().getCurrentPosition();

        //drive forward, clearing front of ramp
        timeout.reset();
        do {
            waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(.8);
            hardware.getRightDriveMotor().setPower(.8);
        } while (hardware.getLeftDriveMotor().getCurrentPosition() < encoderTarget && timeout.time() < 3);

        //stop motors
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        //encoder target
        encoderTarget = hardware.getLeftDriveMotor().getCurrentPosition() - 700;

        //back up for turn
        timeout.reset();
        do {
            waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(-.5);
            hardware.getRightDriveMotor().setPower(-.5);
        } while (hardware.getLeftDriveMotor().getCurrentPosition() > encoderTarget && timeout.time() < 1);

        //stop motors
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        //curve around to face the ramp
        gyroTarget = 85;
        if (blueSide) {
            gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + gyroTarget;
        } else {
            gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget;
        }

        do {
            waitForNextHardwareCycle();
            int headingError = (gyroTarget - hardware.getRobotRotationGyro().getIntegratedZValue());
            double drivePower = headingError / 130.0;
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < .15 && drivePower != 0) {
                drivePower = (.15 * drivePower / Math.abs(drivePower));
            }
            hardware.getLeftDriveMotor().setPower(drivePower);
            hardware.getRightDriveMotor().setPower(-drivePower);
            telemetry.addData("error", headingError);
            telemetry.addData("target, current", gyroTarget + ", " + hardware.getRobotRotationGyro().getIntegratedZValue());
            telemetry.addData("Motor power", drivePower);
        } while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget) > 1 && timeout.time() < 5);

        //stop motors
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        // TODO: 2/14/2016 Raise churro grabber here

        //drive up ramp
        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(2000);

        // TODO: 2/14/2016 Lower churro grabber here

        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        //extend arm
        hardware.getArmTelescopeMotors().setPower(.25);
        timeout.reset();
        while (hardware.getArmTelescopeMotors().getCurrentPosition() > -2500 && timeout.time() < 2)
            waitForNextHardwareCycle();
        hardware.getArmTelescopeMotors().setPowerFloat();
    }
}