package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/12/2015.
 */
public class AutoBlue extends LinearOpMode{

    RobotHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(hardwareMap);

        waitForStart();

        hardware.getLeftDriveMotor().setPower(1);
        hardware.getRightDriveMotor().setPower(1);
        sleep(400);

        hardware.getRightDriveMotor().setPower(0);

        sleep(2000);

        hardware.getRightDriveMotor().setPower(1);

        sleep(750);
        hardware.getLeftDriveMotor().setPower(0);
        hardware.getRightDriveMotor().setPower(0);
    }
}