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

        //extend arm
        autoFunctions.extendArm();

        waitForNextHardwareCycle();
        sleep(500);
        waitForNextHardwareCycle();


        autoFunctions.driveStraight(8500, AutoFunctions.DriveStraightDirection.FORWARD, .5);

        //send any late signals
        waitForNextHardwareCycle();
        sleep(500);
        waitForNextHardwareCycle();

        //turn to face beacon
        int angle = 43;
        if (blueSide) {
            autoFunctions.twoWheelTurn(angle);
        } else {
            autoFunctions.twoWheelTurn(-angle);
        }

        waitForNextHardwareCycle();
        sleep(500);
        waitForNextHardwareCycle();

        //raise arm
        int target = 1100 + hardware.getArmPivotMotors().getCurrentPosition();
        timeout.reset();
        hardware.getArmPivotMotors().setPower(.3);
        while (hardware.getArmPivotMotors().getCurrentPosition() < target && timeout.time() < 3)
            waitForNextHardwareCycle();
        hardware.getArmPivotMotors().setPower(0);

        waitForNextHardwareCycle();
        sleep(500);
        waitForNextHardwareCycle();

        autoFunctions.driveStraight(1500, AutoFunctions.DriveStraightDirection.FORWARD, .25);

        waitForNextHardwareCycle();
        sleep(500);
        waitForNextHardwareCycle();

        hardware.getArmIntakeMotor().setPower(-.12);

        waitForNextHardwareCycle();
        sleep(1500);
        waitForNextHardwareCycle();

        hardware.getArmIntakeMotor().setPowerFloat();
    }
}
