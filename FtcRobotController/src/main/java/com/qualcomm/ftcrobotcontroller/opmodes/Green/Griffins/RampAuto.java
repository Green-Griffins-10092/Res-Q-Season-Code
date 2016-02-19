package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by David on 2/14/2016.
 * The master autonomous for the ramp.
 */
public abstract class RampAuto extends LinearOpMode {

    protected static boolean blueSide = true;
    protected static int wait = 0;
    RobotHardware hardware;

    public void oneWheelTimedTurn( DcMotor turningMotor, long milliseconds) throws InterruptedException {
        waitForNextHardwareCycle();
        long startTime = System.currentTimeMillis();

        double power = 1.0;
        turningMotor.setPower(power);
        sleep( milliseconds - ((long) (milliseconds*.1)));

        while( System.currentTimeMillis() - startTime < milliseconds )
        {
            RobotLog.i( "1wheel difftime: " + (System.currentTimeMillis() - startTime) +" power: " + power);
            power = power * .5;
            turningMotor.setPower( power );
            sleep( 50 );
        }
        turningMotor.setPower( 0 );
        //send any late signals
        waitForNextHardwareCycle();
    }
    enum TwoWheelTurnDirection { left, right };

    public void twoWheelTimedTurn( TwoWheelTurnDirection direction, long milliseconds ) throws InterruptedException {
        waitForNextHardwareCycle();
        long startTime = System.currentTimeMillis();

        double leftPower, rightPower;
        if( direction == TwoWheelTurnDirection.left ) {
            leftPower = 1.0;
            rightPower = -1.0;
        } else {
            leftPower = -1.0;
            rightPower = 1.0;
        }
        hardware.getLeftDriveMotor().setPower(leftPower);
        hardware.getRightDriveMotor().setPower(rightPower);
        sleep(milliseconds - ((long) (milliseconds * .1)));

        while( System.currentTimeMillis() - startTime < milliseconds )
        {
            RobotLog.i( "2wheel difftime: " + (System.currentTimeMillis() - startTime) +" power: " + leftPower);
            leftPower = leftPower * .5;
            rightPower = rightPower * .5;

            hardware.getLeftDriveMotor().setPower( leftPower);
            hardware.getRightDriveMotor().setPower( rightPower );
            sleep( 50 );
        }
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);
        //send any late signals
        waitForNextHardwareCycle();

    }

    public void oneWheelTurn(DcMotor turningMotor, double angle) throws InterruptedException {
        double minimumPower = .2;
        int gyroTarget = hardware.getRobotRotationGyro().getIntegratedZValue() + (int) angle;

        ElapsedTime timeout = new ElapsedTime();
        double drivePower = 1.0;
        do {
            waitForNextHardwareCycle();
            int headingError = (gyroTarget - hardware.getRobotRotationGyro().getIntegratedZValue());

            RobotLog.i("time: " + timeout.time());
            RobotLog.i("error: " + headingError);
            RobotLog.i("Motor power: " + drivePower);
            RobotLog.i("-----------------------");
            turningMotor.setPower(drivePower);
            drivePower = headingError / (2*angle);
            drivePower = Range.clip(drivePower, -1, 1);
            if (Math.abs(drivePower) < minimumPower) {
                drivePower = minimumPower * Math.signum(drivePower);
            }
            telemetry.addData("error", headingError);
            telemetry.addData("Motor power", drivePower);
        }
        while (Math.abs(hardware.getRobotRotationGyro().getIntegratedZValue() - gyroTarget) > 0 && timeout.time() < 5);

        //stop driving
        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();

    }

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
            drivePower = headingError / (2*Math.abs(angle));
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

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timeout = new ElapsedTime();

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

//        oneWheelTimedTurn( hardware.getLeftDriveMotor(), 640 );
//        sleep(1000);

        int angle = 39;

        if (blueSide) {
            oneWheelTurn(hardware.getLeftDriveMotor(), angle);
        } else {
            oneWheelTurn(hardware.getRightDriveMotor(), -angle);
        }
        sleep(1000);

        //encoder target
        int encoderTarget = 4700 + hardware.getLeftDriveMotor().getCurrentPosition();

        //drive forward, clearing front of ramp
        timeout.reset();
        do {
            waitForNextHardwareCycle();
            hardware.getLeftDriveMotor().setPower(.5);
            hardware.getRightDriveMotor().setPower(.5);
        }
        while (hardware.getLeftDriveMotor().getCurrentPosition() < encoderTarget && timeout.time() < 3);

        //stop motors
        waitForNextHardwareCycle();
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
        }
        while (hardware.getLeftDriveMotor().getCurrentPosition() > encoderTarget && timeout.time() < 1);

        //stop motors
        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(1000);

        double twoWheelTurnAngle = 85;
        if (blueSide) {
            twoWheelTurn(twoWheelTurnAngle);
        } else {
            twoWheelTurn(-twoWheelTurnAngle);
        }
        sleep(1000);

        // TODO: 2/14/2016 Raise churro grabber here

        //drive up ramp
        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);

        //send any late signals
        waitOneFullHardwareCycle();
        sleep(2000);

        // TODO: 2/14/2016 Lower churro grabber here

        waitForNextHardwareCycle();
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
        waitForNextHardwareCycle();
    }
}