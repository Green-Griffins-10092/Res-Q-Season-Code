package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 1/1/2016.
 * This is the teleop opmode
 * <p/>
 * Controls:
 *  Gamepad 1:
 *      tank drive on joysticks
 *  Gamepad 2:
 *      turret pivot on left x axis
 *      arm pivot on right y axis
 *      arm telescope on left y axis
 *      arm intake is default to 1
 *      right bumper stops the arm intake
 *      right trigger controls input
 */

public class Teleop extends OpMode {

    public static final int ENCODER_COUNTS_PER_ROTATION = 1120;
    public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS = 3;
    public static final double ENCODER_COUNTS_PER_DEGREE = ENCODER_COUNTS_PER_ROTATION / MOTOR_ROTATIONS_PER_TURRET_ROTATIONS / 360;
    public static final int TURRET_PIVOT_DEGREE_LIMIT = 270;

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

        //turret pivot on gamepad 2, left x axis, with limits
        DcMotor turretMotor = hardware.getTurretPivotMotor();
        if (turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_DEGREE - 10) {
            turretMotor.setPower(Range.clip(gamepad2.left_stick_y, -1, 0));
        } else if (-turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_DEGREE - 10) {
            turretMotor.setPower(Range.clip(gamepad2.left_stick_y, 0, 1));
        } else {
            turretMotor.setPower(gamepad2.left_stick_x);
        }

        //arm telescope on gamepad 2, left y axis
        //arm pivot on gamepad 2, right y axis
        hardware.getArmTelescopeMotors().setPower(-gamepad2.left_stick_y);
        hardware.getArmPivotMotors().setPower(-gamepad2.right_stick_y);

        if (gamepad2.right_bumper) {
            hardware.getArmIntakeMotor().setPower(0);
        } else if (gamepad2.right_trigger != 0) {
            hardware.getArmIntakeMotor().setPower(-gamepad2.right_trigger);
        } else {
            hardware.getArmIntakeMotor().setPower(1);
        }

        telemetry.addData("Gamepad 1", gamepad1.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 1 (start+a)" : gamepad1);
        telemetry.addData("Gamepad 2", gamepad2.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 2 (start+b)" : gamepad2);
        telemetry.addData("Time(elapsed:left)", getRuntime() + ":" + (90 - getRuntime()));
        telemetry.addData("Turret Position(encoder counts:degrees)", hardware.getTurretPivotMotor().getCurrentPosition() +
                ":" + hardware.getTurretPivotMotor().getCurrentPosition() / ENCODER_COUNTS_PER_DEGREE);
    }

    @Override
    public void stop() {
        int turretPivotReading = hardware.getTurretPivotMotor().getCurrentPosition();
        telemetry.addData("Turret Encoder Counts", turretPivotReading);
    }
}