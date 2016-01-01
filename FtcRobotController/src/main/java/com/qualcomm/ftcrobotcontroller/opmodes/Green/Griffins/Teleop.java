package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 1/1/2016.
 * This is the teleop opmode
 */

public class Teleop extends OpMode {

    RobotHardware hardware;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        hardware.getLeftDriveMotor().setPower(-gamepad1.left_stick_y);
        hardware.getRightDriveMotor().setPower(-gamepad1.right_stick_y);
    }

    @Override
    public void stop() {
    }
}