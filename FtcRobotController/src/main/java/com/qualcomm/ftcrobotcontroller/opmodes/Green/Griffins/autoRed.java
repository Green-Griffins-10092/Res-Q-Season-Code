package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by David on 12/12/2015.
 */
public class AutoRed extends LinearOpMode{

    RobotHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = RobotHardware.initialize(hardwareMap);

        waitForStart();

        hardware.getLeftDrive().setPower(1);
        hardware.getRightDrive().setPower(1);
        sleep(400);

        hardware.getLeftDrive().setPower(0);

        sleep(3000);

        hardware.getLeftDrive().setPower(0);
        hardware.getRightDrive().setPower(0);
    }
}
