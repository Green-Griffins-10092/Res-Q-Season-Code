package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 1/1/2016.
 * This is the teleop opmode
 * <p/>
 * Controls: see the gamepad map
 *  also, arm pivot is on gamepad 2 left y axis
 *  and arm move to buttons have not been written yet
 */

public class Teleop extends OpMode {

    public static final int ENCODER_COUNTS_PER_ROTATION = 1120;

    public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS = 6;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREES = ENCODER_COUNTS_PER_ROTATION / MOTOR_ROTATIONS_PER_TURRET_ROTATIONS / 360;
    public static final int TURRET_PIVOT_DEGREE_LIMIT = 270;

//    public static final double MOTOR_ROTATIONS_PER_ARM_TELESCOPE_ROTATIONS = 2;
//    public static final double ARM_TELESCOPE_MOTOR_ROTATION_LIMIT = 2;

    RobotHardware hardware;
    int autoArmState;
    /*states:
     * abort = -1
     * manuel = 0
     * reset = 1
     * medium = 2
     * tall = 3
     * extend = 4
     * hang = 5
     */

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
        autoArmState = 0;
    }

    @Override
    public void loop() {
        //tank control, gamepad 1
        hardware.getLeftDriveMotor().setPower(-gamepad1.left_stick_y);
        hardware.getRightDriveMotor().setPower(-gamepad1.right_stick_y);

        if (autoArmState != 0 && gamepad2.b) {
            autoArmState = -1;
        } else if (gamepad2.x) {
            autoArmState = 1;
        } else if (gamepad2.a) {
            autoArmState = 2;
        } else if (gamepad2.y) {
            autoArmState = 3;
        } else if (gamepad1.y) {
            autoArmState = 4;
        } else if (gamepad1.dpad_up) {
            autoArmState = 5;
        }

        switch (autoArmState) {
            case 0:
                //turret pivot on gamepad 2, left x axis and left and right dpad, with limits
                DcMotor turretMotor = hardware.getTurretPivotMotor();
                double turretPower = 0; //for finding the power

                if (gamepad2.dpad_left) {
                    turretPower = -0.25;
                } else if (gamepad2.dpad_right) {
                    turretPower = 0.25;
                } else {
                    turretPower = gamepad2.left_stick_x;
                }
                if (turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
                    turretMotor.setPower(Range.clip(turretPower, -1, 0));
                } else if (-turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
                    turretMotor.setPower(Range.clip(turretPower, 0, 1));
                } else {
                    turretMotor.setPower(turretPower);
                }

                //arm telescope on gamepad 2, left y axis and dpad up and down
                double sliderPower = 0; //find appropriate power
                if (gamepad2.dpad_up) {
                    sliderPower = 0.1;
                } else if (gamepad2.dpad_down) {
                    sliderPower = -0.1;
                } else{
                    sliderPower = gamepad2.left_stick_y;
                }
//                if (hardware.getArmTelescopeMotors().getCurrentPosition() >= ARM_TELESCOPE_MOTOR_ROTATION_LIMIT*ENCODER_COUNTS_PER_ROTATION-10) {
//                    sliderPower = Range.clip(sliderPower, -1, 0);
//                }
                hardware.getArmTelescopeMotors().setPower(sliderPower);

                //arm pivot on gamepad 2, right y axis
                hardware.getArmPivotMotors().setPower(-gamepad2.right_stick_y);
                break;
            case -1:
                hardware.getTurretPivotMotor().setPowerFloat();
                hardware.getArmPivotMotors().setPowerFloat();
                hardware.getArmTelescopeMotors().setPowerFloat();
                autoArmState = 0;
                break;
            default:
                telemetry.addData("WARNING, illegal arm auto state", autoArmState);
                autoArmState = -1;
                break;
        }

        double armIntakePower = 0;
        if (gamepad1.left_trigger != 0) {
            armIntakePower = gamepad1.left_trigger;
        } else if (gamepad2.left_trigger != 0) {
            armIntakePower = gamepad2.left_trigger;
        } else if (gamepad2.left_bumper) {
            armIntakePower = -0.5;
        } else {
            armIntakePower = -gamepad1.right_trigger;
        }
        hardware.getArmIntakeMotor().setPower(armIntakePower);

        telemetry.addData("Gamepad 1", gamepad1.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 1 (start+a)" : gamepad1);
        telemetry.addData("Gamepad 2", gamepad2.id == Gamepad.ID_UNASSOCIATED ? "Connect gamepad 2 (start+b)" : gamepad2);
        telemetry.addData("Time(elapsed:left)", getRuntime() + ":" + (90 - getRuntime()));
        telemetry.addData("Arm auto state", autoArmState);
        telemetry.addData("Turret Position(encoder counts:degrees)", hardware.getTurretPivotMotor().getCurrentPosition() +
                ":" + hardware.getTurretPivotMotor().getCurrentPosition() / ENCODER_COUNTS_PER_TURRET_DEGREES);
    }

    @Override
    public void stop() {
        int turretPivotReading = hardware.getTurretPivotMotor().getCurrentPosition();
        telemetry.addData("Turret Encoder Counts", turretPivotReading);
    }
}