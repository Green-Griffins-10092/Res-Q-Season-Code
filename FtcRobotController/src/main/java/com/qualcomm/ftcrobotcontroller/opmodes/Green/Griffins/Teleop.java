package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 12/1/2015.
 * This is the master Teleop program for the FIRST Res-Q season.
 * See Gamepad map.png for controls.
 */
public class Teleop extends OpMode {

    //the robot hardware
    private RobotHardware hardware;

    //variables that will be used in loop for holding data between iterations
    private boolean locked;
    private int autoArm;

    public static double scale(double input) {
        //TODO: create scaling algorithm, or decide against it
        return input;
    }

    private void sendTelemetryData() {
        telemetry.addData("00", locked ? "Controllers locked" : "Controllers unlocked");

        telemetry.addData("01 left shifter power", hardware.getLeftDrive().getPower());
        telemetry.addData("01 right shifter power", hardware.getRightDrive().getPower());

        telemetry.addData("02 left shifter position", hardware.getLeftDrive().getPosition());
        telemetry.addData("02 right shifter position", hardware.getRightDrive().getPosition());

        telemetry.addData("05 arm intake power", hardware.getArmIntakeMotor().getPower());

        telemetry.addData("99 Gamepad 1", gamepad1.toString());
        telemetry.addData("99 Gamepad 2", gamepad2.toString());
    }

    @Override
    public void init() {
        hardware = RobotHardware.initialize(hardwareMap);

        hardware.getLeftDrive().shift(Shifter.ShifterPosition.DRIVE);
        hardware.getRightDrive().shift(Shifter.ShifterPosition.DRIVE);

        //TODO: set button pusher to center position
        locked = false;
        autoArm = 0;
    }

    @Override
    public void loop() {

        //check lock status first
        if (gamepad1.back)
            locked = true;
        else if (gamepad1.start)
            locked = false;

        //control code
        if (!locked) {

            // create a reference for the shifters
            Shifter leftShifter, rightShifter;
            leftShifter = hardware.getLeftDrive();
            rightShifter = hardware.getRightDrive();

            /*
            makes sure that motors are not drive while shifting if this is true,
            a shifting operation is going to occur at the end of the loop,
            and it is not safe to command the motors to move
            */
            boolean shifting = false;

            //shifting
            if (gamepad1.left_bumper && gamepad1.left_trigger == 1 &&
                    leftShifter.getPosition() == Shifter.ShifterPosition.NEUTRAL) {
                leftShifter.shift(Shifter.ShifterPosition.NEUTRAL);
                rightShifter.shift(Shifter.ShifterPosition.NEUTRAL);
                shifting = true;
            } else if (gamepad1.left_trigger == 1 &&
                    leftShifter.getPosition() == Shifter.ShifterPosition.DRIVE) {
                leftShifter.shift(Shifter.ShifterPosition.DRIVE);
                rightShifter.shift(Shifter.ShifterPosition.DRIVE);
                shifting = true;
            } else if (gamepad1.left_bumper &&
                    leftShifter.getPosition() == Shifter.ShifterPosition.ARM) {
                leftShifter.shift(Shifter.ShifterPosition.ARM);
                rightShifter.shift(Shifter.ShifterPosition.ARM);
                shifting = true;
            }

            //these values will be overwritten if needed
            rightShifter.setPowerFloat();
            rightShifter.setPowerFloat();

            //tank drive if shifted to drive
            if (leftShifter.getPosition() == Shifter.ShifterPosition.DRIVE && !shifting) {
                leftShifter.setPower(scale(gamepad1.left_stick_y));
                rightShifter.setPower(scale(gamepad1.right_stick_y));
            }

            //drive the arm intake motor
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                hardware.getArmIntakeMotor().setPower(-1);
            } else if (gamepad1.right_trigger != 0 || gamepad2.right_trigger != 0) {
                if (gamepad2.right_trigger != 0)
                    hardware.getArmIntakeMotor().setPower(gamepad2.right_trigger);
                else
                    hardware.getArmIntakeMotor().setPower(gamepad1.right_trigger);
            } else {
                hardware.getArmIntakeMotor().setPower(0);
            }

            if (autoArm == 0)
            {
                //we are under manual control
                if (gamepad2.dpad_down){
                    if (leftShifter.getPosition() == Shifter.ShifterPosition.ARM && !shifting){
                        leftShifter.setPower(-.01);
                        rightShifter.setPower(-.01);
                    }
                    hardware.getSpoolMotor().setPower(-.01);
                } else if (gamepad2.dpad_up){
                    if (leftShifter.getPosition() == Shifter.ShifterPosition.ARM && !shifting){
                        leftShifter.setPower(.01);
                        rightShifter.setPower(.01);
                    }
                    hardware.getSpoolMotor().setPower(.01);
                }else{
                    if (leftShifter.getPosition() == Shifter.ShifterPosition.ARM && !shifting) {
                        leftShifter.setPower(gamepad2.left_stick_y);
                        rightShifter.setPower(gamepad2.left_stick_y);
                    }
                    hardware.getSpoolMotor().setPower(gamepad2.left_stick_y);
                }
            }
            else{
                //we are on automatic arm control
                if (gamepad2.b){
                    //kill auto routine
                    autoArm = 0;
                    hardware.getSpoolMotor().setPowerFloat();
                    if (leftShifter.getPosition() == Shifter.ShifterPosition.ARM){
                        leftShifter.setPowerFloat();
                        rightShifter.setPowerFloat();
                    }
                }else {
                    //TODO:put if statements for automatic arm control
                }
            }

        }//end if statement (!locked)

        //add sendTelemetryData data
        sendTelemetryData();

    }//end loop method
}
