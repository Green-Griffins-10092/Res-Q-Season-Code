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

    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 = 1680;
    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_40 = 1120;

    public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS = 6;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREES = ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 * MOTOR_ROTATIONS_PER_TURRET_ROTATIONS / 360;
    public static final int TURRET_PIVOT_DEGREE_LIMIT = 270;

    public static final double MOTOR_ROTATIONS_PER_ARM_TELESCOPE_ROTATIONS = 2;
    public static final double ARM_TELESCOPE_MOTOR_ROTATION_LIMIT = 2;

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

    public double ramp(double target, double current, double rampSpeed) {
        double rtn = current;
        rtn += target/rampSpeed;
        rtn /= (rampSpeed+1)/rampSpeed;
        return rtn;
    }

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        telemetry.setSorted(false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        resetStartTime();
        autoArmState = 0;
    }

    @Override
    public void loop() {
        //Gamepad 2 full override
        final boolean GAMEPAD_2_OVERRIDE = gamepad2.right_trigger == 1;

        //tank control, gamepad 1
        double leftDrivePower = -gamepad1.left_stick_y;
        double rightDrivePower = -gamepad1.right_stick_y;
        if (gamepad1.left_bumper) {
            leftDrivePower = Range.scale(leftDrivePower, -1, 1, -.5, .5);
            rightDrivePower = Range.scale(rightDrivePower, -1, 1, -.5, .5);
        }
//        leftDrivePower = ramp(leftDrivePower, hardware.getLeftDriveMotor().getPower(), 5);
//        rightDrivePower = ramp(rightDrivePower, hardware.getRightDriveMotor().getPower(), 5);
        hardware.getLeftDriveMotor().setPower(leftDrivePower);
        hardware.getRightDriveMotor().setPower(rightDrivePower);

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
                double turretPower; //for finding the power
                if (gamepad2.dpad_left) {
                    turretPower = -0.5;
                } else if (gamepad2.dpad_right) {
                    turretPower = 0.5;
                } else {
                    turretPower = gamepad2.left_stick_x;
                }
                if (!GAMEPAD_2_OVERRIDE) {
                    if (turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
                        turretPower = Range.clip(turretPower, -1, 0);
                    } else if (-turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
                        turretPower = Range.clip(turretPower, 0, 1);
                    }
                }
                turretPower = ramp(turretPower, turretMotor.getPower(), 5);
                turretMotor.setPower(turretPower);

                //arm telescope on gamepad 2, left y axis and dpad up and down
                double sliderPower; //find appropriate power
                if (gamepad2.dpad_up) {
                    sliderPower = 0.1;
                } else if (gamepad2.dpad_down) {
                    sliderPower = -0.1;
                } else{
                    sliderPower = gamepad2.left_stick_y;
                }
//                if (hardware.getArmTelescopeMotors().getCurrentPosition() >= ARM_TELESCOPE_MOTOR_ROTATION_LIMIT*ENCODER_COUNTS_PER_ROTATION_NEVEREST_40-10 && !GAMEPAD_2_OVERRIDE) {
//                    sliderPower = Range.clip(sliderPower, -1, 0);
//                } else if (hardware.getArmTelescopeMotors().getCurrentPosition() >= ARM_TELESCOPE_MOTOR_ROTATION_LIMIT*ENCODER_COUNTS_PER_ROTATION_NEVEREST_40-10 && !GAMEPAD_2_OVERRIDE) {
//                    sliderPower = Range.clip(sliderPower, 0, 1);
//                }
                hardware.getArmTelescopeMotors().setPower(sliderPower);

                //arm pivot on gamepad 2, right y axis
                double armPivotPower = -gamepad2.right_stick_y;
                if (!GAMEPAD_2_OVERRIDE) {
                    armPivotPower = Range.scale(armPivotPower, -1, 1, -.25, .25);
                }
                hardware.getArmPivotMotors().setPower(armPivotPower);
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

        double armIntakePower;
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
        double time = ((int)(100*getRuntime()))/100.0;
        telemetry.addData("Time(elapsed:left)", time + ":" + (90 - time));
        telemetry.addData("Arm auto state", autoArmState);
        telemetry.addData("Turret Position(encoder counts:degrees)", hardware.getTurretPivotMotor().getCurrentPosition() +
                ":" + hardware.getTurretPivotMotor().getCurrentPosition() / ENCODER_COUNTS_PER_TURRET_DEGREES);
        telemetry.addData("Pivot encoder count", hardware.getArmPivotMotors().getCurrentPosition());
        telemetry.addData("Telescope encoder count", hardware.getArmTelescopeMotors().getCurrentPosition());
    }

    @Override
    public void stop() {
        int turretPivotReading = hardware.getTurretPivotMotor().getCurrentPosition();
        telemetry.addData("Turret Encoder Counts", turretPivotReading);
    }
}