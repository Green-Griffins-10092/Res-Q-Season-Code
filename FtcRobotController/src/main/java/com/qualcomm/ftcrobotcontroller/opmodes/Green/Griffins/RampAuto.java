package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by David on 2/14/2016.
 * The master autonomous for the ramp.
 */
public abstract class RampAuto extends LinearOpMode {

    protected static boolean blueSide = true;
    protected static int waitInMilliseconds = 0;
    RobotHardware hardware;
    AutoFunctions autoFunctions;

    public void oneWheelTimedTurn( DcMotor turningMotor, long milliseconds) throws InterruptedException {
        waitForNextHardwareCycle();
        long startTime = System.currentTimeMillis();

        double power = 1.0;
        turningMotor.setPower(power);
        sleep(milliseconds - ((long) (milliseconds * .1)));

        while( System.currentTimeMillis() - startTime < milliseconds )
        {
            RobotLog.i( "1wheel difftime: " + (System.currentTimeMillis() - startTime) +" power: " + power);
            power = power * .5;
            turningMotor.setPower( power );
            sleep( 50 );
        }
        turningMotor.setPower(0);
        //send any late signals
        waitForNextHardwareCycle();
    }

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

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timeout = new ElapsedTime();

        hardware = new RobotHardware(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

        hardware.getRobotRotationGyro().calibrate();

        hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        hardware.setPanelPosition(RobotHardware.PanelPosition.PANEL_LOWERED);

        sleep(waitInMilliseconds);

        while (hardware.getRobotRotationGyro().isCalibrating()) {
            waitForNextHardwareCycle();
        }

        int angle = 39;
        if (blueSide) {
            autoFunctions.oneWheelTurn(hardware.getLeftDriveMotor(), angle);
        } else {
            autoFunctions.oneWheelTurn(hardware.getRightDriveMotor(), -angle);
        }
        sleep(500);

        //drive forward, clearing front of ramp
        autoFunctions.driveStraight(4700, AutoFunctions.DriveStraightDirection.FORWARD, .5);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(500);

        //back up
        autoFunctions.driveStraight(700, AutoFunctions.DriveStraightDirection.BACKWARD, .5);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(500);

        double twoWheelTurnAngle = 85;
        if (blueSide) {
            autoFunctions.twoWheelTurn(twoWheelTurnAngle);
        } else {
            autoFunctions.twoWheelTurn(-twoWheelTurnAngle);
        }

        sleep(500);
        waitForNextHardwareCycle();

        hardware.setPanelPosition(RobotHardware.PanelPosition.PANEL_RAISED);

        sleep(500);
        waitOneFullHardwareCycle();

        //drive up ramp
        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);

        //send any late signals
        waitOneFullHardwareCycle();
        sleep(2000);
        waitForNextHardwareCycle();

        hardware.setPanelPosition(RobotHardware.PanelPosition.PANEL_LOWERED);
        waitOneFullHardwareCycle();

        sleep(500);

        waitForNextHardwareCycle();
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(500);

        autoFunctions.extendArm();
    }

    enum TwoWheelTurnDirection {left, right}
}