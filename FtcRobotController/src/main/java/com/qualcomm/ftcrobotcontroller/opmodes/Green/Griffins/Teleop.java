package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by David on 1/1/2016.
 * This is the teleop opmode
 *
 * Controls:
 *  Gamepad 1:
 *      tank drive on joysticks
 *  Gamepad 2:
 *      turret pivot on left x axis
 *      arm pivot on right y axis
 *      arm telescope on left y axis
 */

public class Teleop extends OpMode {

    RobotHardware hardware;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Gamepad 1", gamepad1.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 1 (start+a)" : gamepad1);
        telemetry.addData("Gamepad 2", gamepad2.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 2 (start+b)" : gamepad2);
    }

    @Override
    public void start() {
        resetStartTime();
    }

    @Override
    public void loop() {
        //tank control, gamepad 1
        hardware.getLeftDriveMotor().setPower(-gamepad1.left_stick_y);
        hardware.getRightDriveMotor().setPower(-gamepad1.right_stick_y);

        //turret pivot on gamepad 2, left x axis
        //arm telescope on gamepad 2, left y axis
        //arm pivot on gamepad 2, right y axis
        hardware.getTurretPivotMotor().setPower(gamepad2.left_stick_x);
        hardware.getArmTelescopeMotors().setPower(-gamepad2.left_stick_y);
        hardware.getArmPivotMotors().setPower(-gamepad2.right_stick_y);

        if (gamepad2.right_bumper) {
            hardware.getArmIntakeMotor().setPower(.5);
        } else {
            hardware.getArmIntakeMotor().setPower(1);
        }

        telemetry.addData("Gamepad 1", gamepad1.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 1 (start+a)" : gamepad1);
        telemetry.addData("Gamepad 2", gamepad2.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 2 (start+b)" : gamepad2);
        telemetry.addData("Time(elapsed:left)", getRuntime() + ":" + (90-getRuntime()) );
    }

    @Override
    public void stop() {
        int turretPivotReading = hardware.getTurretPivotMotor().getCurrentPosition();
        telemetry.addData("Turret Encoder Counts", turretPivotReading);
    }
}