package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by David on 3/30/2016.
 * For use at outreaches
 */
public class OutreachTeleop extends OpMode {

    public static final int GAMEPAD_1_FULL_CONTROL = 0;
    public static final int GAMEPAD_2_OVERRIDE = 1;
    public static final int GAMEPAD_1_MUTE = 2;
    /*
     * states:
     *  0 : gamepad 1 control has full control of the base of the robot.
     *      gamepad 2 has arm and turret control.e
     *  1 : gamepad 2 will override gamepad 1 if there is any input from gamepad 2.
     *          Not Implemented Yet!
     *  2 : gamepad 1 muted, gamepad 2 has full control of the drive base.
     * see constants below
     */
    int overrideState;
    RobotHardware hardware;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        telemetry.setSorted(false);
    }

    @Override
    public void start() {
        resetStartTime();
        if (gamepad2 == null || gamepad2.id == Gamepad.ID_UNASSOCIATED) {
            overrideState = GAMEPAD_1_FULL_CONTROL;
        } else {
            overrideState = GAMEPAD_1_MUTE;
        }
    }

    @Override
    public void loop() {

        if (gamepad2.b) {
            overrideState = GAMEPAD_1_MUTE;
        } else if (gamepad2.x) {
            overrideState = GAMEPAD_2_OVERRIDE;
        } else if (gamepad2.a) {
            overrideState = GAMEPAD_1_FULL_CONTROL;
        }

        if (gamepad2.start) {
            resetStartTime();
        }

        switch (overrideState) {
            case GAMEPAD_1_FULL_CONTROL:

                //drive code
                double throttle = -gamepad1.left_stick_y;
                double direction = gamepad1.left_stick_x;

                double leftDrivePower = throttle + direction;
                double rightDrivePower = throttle - direction;

                leftDrivePower = Range.clip(leftDrivePower, -1, 1);
                rightDrivePower = Range.clip(rightDrivePower, -1, 1);

                leftDrivePower = Range.scale(leftDrivePower, -1, 1, -.75, .75);
                rightDrivePower = Range.scale(rightDrivePower, -1, 1, -.75, .75);

                hardware.getLeftDriveMotor().setPower(leftDrivePower);
                hardware.getRightDriveMotor().setPower(rightDrivePower);

                //flapper code
                double flapperPower = -gamepad1.right_trigger;
                if (gamepad1.right_bumper) {
                    flapperPower = 1;
                }
                hardware.getArmIntakeMotor().setPower(flapperPower);

                //gamepad 2
                DcMotor turretMotor = hardware.getTurretPivotMotor();
                double turretPower; //for finding the power
                if (gamepad2.dpad_left) {
                    turretPower = -0.5;
                } else if (gamepad2.dpad_right) {
                    turretPower = 0.5;
                } else {
                    turretPower = gamepad2.left_stick_x;
                }

                if (gamepad2.left_stick_y < -.5) {
                    turretPower = 0;
                }
                turretMotor.setPower(turretPower);

                //arm telescope on gamepad 2, left y axis and dpad up and down
                double sliderPower; //find appropriate power
                if (gamepad2.dpad_up) {
                    sliderPower = 0.1;
                } else if (gamepad2.dpad_down) {
                    sliderPower = -0.1;
                } else {
                    sliderPower = -gamepad2.left_stick_y;
                }
                hardware.getArmTelescopeMotors().setPower(sliderPower);

                //arm pivot on gamepad 2, right y axis
                double armPivotPower = -gamepad2.right_stick_y;
                hardware.getArmPivotMotors().setPower(armPivotPower);

                if (gamepad2.left_trigger != 0 && flapperPower == 0) {
                    flapperPower = gamepad2.left_trigger;
                } else if (gamepad2.right_trigger != 0 && flapperPower == 0) {
                    flapperPower = -gamepad2.right_trigger;
                }
                hardware.getArmIntakeMotor().setPower(flapperPower);

                break;

            case GAMEPAD_1_MUTE:
                throttle = -gamepad2.left_stick_y;
                direction = gamepad2.left_stick_x;

                leftDrivePower = throttle + direction;
                rightDrivePower = throttle - direction;

                leftDrivePower = Range.clip(leftDrivePower, -1, 1);
                rightDrivePower = Range.clip(rightDrivePower, -1, 1);

                hardware.getLeftDriveMotor().setPower(leftDrivePower);
                hardware.getRightDriveMotor().setPower(rightDrivePower);

                //flapper code
                flapperPower = -gamepad2.right_trigger;
                if (gamepad2.right_bumper) {
                    flapperPower = 1;
                }

                hardware.getArmIntakeMotor().setPower(flapperPower);
                break;

            default:
                overrideState = GAMEPAD_1_MUTE;
        }

        telemetry.addData("Time (seconds)", (int) getRuntime());
        telemetry.addData("Override state", overrideState);
    }
}
