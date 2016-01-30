package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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

    public static final int TURRET_PIVOT_DEGREE_LIMIT = 270;

    //in inches
    public static final double ARM_TELESCOPE_EXTENSION_LIMIT = 12;

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


    public double deltaTheta(double deltaX, double deltaY, double currentTheta, double currentR) {
        double deltaTheta;

        double currentX = currentR * Math.cos(Math.toRadians(currentTheta));
        double currentY = currentR * Math.sin(Math.toRadians(currentTheta));

        double newX = deltaX + currentX;
        double newY = deltaY + currentY;

        deltaTheta = Math.atan(newY/newX) - Math.atan(currentY/currentX);

        return deltaTheta;
    }

    public double deltaR(double deltaX, double deltaY, double currentTheta, double currentR) {
        double deltaR;

        double currentX = currentR * Math.cos(Math.toRadians(currentTheta));
        double currentY = currentR * Math.sin(Math.toRadians(currentTheta));

        double newX = deltaX + currentX;
        double newY = deltaY + currentY;

        deltaR = Math.sqrt(newX*newX + newY*newY) - Math.sqrt(currentX*currentX + currentY*currentY);

        return deltaR;
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
        final boolean GAMEPAD_2_OVERRIDE = (gamepad2.right_trigger == 1);

        //tank control, gamepad 1
        double rightDrivePower;
        double leftDrivePower;
        if (gamepad1.right_bumper) {
            leftDrivePower = .1;
            rightDrivePower = .1;
        } else {
            leftDrivePower = -gamepad1.left_stick_y;
            rightDrivePower = -gamepad1.right_stick_y;
        }
        if (gamepad1.left_bumper) {
            leftDrivePower = Range.scale(leftDrivePower, -1, 1, -.5, .5);
            rightDrivePower = Range.scale(rightDrivePower, -1, 1, -.5, .5);
        }
//        leftDrivePower = ramp(leftDrivePower, hardware.getLeftDriveMotor().getPower(), 5);
//        rightDrivePower = ramp(rightDrivePower, hardware.getRightDriveMotor().getPower(), 5);
        hardware.getLeftDriveMotor().setPower(leftDrivePower);
        hardware.getRightDriveMotor().setPower(rightDrivePower);

        //switch auto state
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

        //reset all encoder values
        if (GAMEPAD_2_OVERRIDE && gamepad2.b) {
            hardware.getArmTelescopeMotors().setMode(DcMotorController.RunMode.RESET_ENCODERS);
            hardware.getLeftDriveMotor().setMode(DcMotorController.RunMode.RESET_ENCODERS);
            hardware.getRightDriveMotor().setMode(DcMotorController.RunMode.RESET_ENCODERS);
            hardware.getArmPivotMotors().setMode(DcMotorController.RunMode.RESET_ENCODERS);
            hardware.getArmIntakeMotor().setMode(DcMotorController.RunMode.RESET_ENCODERS);
            hardware.getTurretPivotMotor().setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }

        switch (autoArmState) {
            case 0:
                //cartesian coordinate controls
                if (gamepad2.right_bumper) {
                    DcMotor turretMotor = hardware.getTurretPivotMotor();
                    SyncedDcMotors armTelescopeMotors = hardware.getArmTelescopeMotors();
                    turretMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    armTelescopeMotors.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                    turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
                    armTelescopeMotors.setTargetPosition(armTelescopeMotors.getCurrentPosition());

                    //these measurements are in degrees and inches
                    double deltaX = gamepad2.left_stick_x * 10;
                    double deltaY = -gamepad2.left_stick_y * 10;
                    double theta = turretMotor.getCurrentPosition() / RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREES;
                    double r = armTelescopeMotors.getCurrentPosition() / RobotHardware.ENCODER_COUNTS_PER_ARM_INCHES;
                    double deltaR = deltaR(deltaX, deltaY, theta, r);
                    double deltaTheta = deltaTheta(deltaX, deltaY, theta, r);

                    if (deltaR+r < ARM_TELESCOPE_EXTENSION_LIMIT) {
                        turretMotor.setTargetPosition((int) ((r+deltaR)*RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREES));
                        armTelescopeMotors.setTargetPosition((int) ((theta+deltaTheta)*RobotHardware.ENCODER_COUNTS_PER_ARM_INCHES));
                        turretMotor.setPower(.75);
                        armTelescopeMotors.setPower(.5);
                    }

                } else { //polar coordinate controls
                    hardware.getArmTelescopeMotors().setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                    hardware.getTurretPivotMotor().setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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
                        if (turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
                            turretPower = Range.clip(turretPower, -1, 0);
                        } else if (-turretMotor.getCurrentPosition() > TURRET_PIVOT_DEGREE_LIMIT * RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREES - 10) {
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
                    } else {
                        sliderPower = -gamepad2.left_stick_y;
                    }
//                if (hardware.getArmTelescopeMotors().getCurrentPosition() >= ARM_TELESCOPE_MOTOR_ROTATION_LIMIT*ENCODER_COUNTS_PER_ROTATION_NEVEREST_40-10 && !GAMEPAD_2_OVERRIDE) {
//                    sliderPower = Range.clip(sliderPower, -1, 0);
//                } else if (hardware.getArmTelescopeMotors().getCurrentPosition() >= ARM_TELESCOPE_MOTOR_ROTATION_LIMIT*ENCODER_COUNTS_PER_ROTATION_NEVEREST_40-10 && !GAMEPAD_2_OVERRIDE) {
//                    sliderPower = Range.clip(sliderPower, 0, 1);
//                }
                    hardware.getArmTelescopeMotors().setPower(sliderPower);
                }
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
                ":" + hardware.getTurretPivotMotor().getCurrentPosition() / RobotHardware.ENCODER_COUNTS_PER_TURRET_DEGREES);
        telemetry.addData("Pivot encoder count", hardware.getArmPivotMotors().getCurrentPosition());
        telemetry.addData("Telescope encoder count", hardware.getArmTelescopeMotors().getCurrentPosition());
    }

    @Override
    public void stop() {
        int turretPivotReading = hardware.getTurretPivotMotor().getCurrentPosition();
        telemetry.addData("Turret Encoder Counts", turretPivotReading);
    }
}