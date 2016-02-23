package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by David on 2/5/2016.
 * The climber auto base class
 */

public abstract class ClimberAuto extends LinearOpMode {

    protected static boolean blueSide = true;
    protected static int waitInMilliseconds = 0;
    protected RobotHardware hardware;
    protected AutoFunctions autoFunctions;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(hardwareMap);
        autoFunctions = new AutoFunctions(hardware, this);

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
            autoFunctions.twoWheelTurn(angle);
        } else {
            autoFunctions.twoWheelTurn(-angle);
        }

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        autoFunctions.driveStraight(1700, AutoFunctions.DriveStraightDirection.FORWARD, .5);

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();

        autoFunctions.driveStraight(1700, AutoFunctions.DriveStraightDirection.BACKWARD, .5);

        waitForNextHardwareCycle();
        sleep(1000);
        waitForNextHardwareCycle();
    }
}
