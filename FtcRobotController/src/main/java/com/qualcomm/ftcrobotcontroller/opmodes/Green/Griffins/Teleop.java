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

    public static double scale(double input) {
        //TODO: create scaling algorithm, or decide against it
        return input;
    }

    @Override
    public void init() {
        hardware = RobotHardware.initialize(hardwareMap);

        hardware.getLeftDrive().shift(Shifter.ShifterPosition.DRIVE);
        hardware.getRightDrive().shift(Shifter.ShifterPosition.DRIVE);

        //TODO: set button pusher to center position
        locked = true;
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

            // create a reference
            Shifter leftShifter, rightShifter;
            leftShifter = hardware.getLeftDrive();
            rightShifter = hardware.getRightDrive();

            //shifting
            if (gamepad1.left_bumper && gamepad1.left_trigger == 1) {
                leftShifter.shift(Shifter.ShifterPosition.NEUTRAL);
                rightShifter.shift(Shifter.ShifterPosition.NEUTRAL);
            } else if (gamepad1.left_trigger == 1) {
                leftShifter.shift(Shifter.ShifterPosition.DRIVE);
                rightShifter.shift(Shifter.ShifterPosition.DRIVE);
            } else if (gamepad1.left_bumper) {
                leftShifter.shift(Shifter.ShifterPosition.ARM);
                rightShifter.shift(Shifter.ShifterPosition.ARM);
            }

            //these values will be overwritten if needed
            rightShifter.setPowerFloat();
            rightShifter.setPowerFloat();

            //tank drive if shifted to drive
            if (leftShifter.getPosition() == Shifter.ShifterPosition.DRIVE
                    && rightShifter.getPosition() == Shifter.ShifterPosition.DRIVE) {
                leftShifter.setPower(scale(gamepad1.left_stick_y));
                rightShifter.setPower(scale(gamepad1.right_stick_y));
            }

        }//end if statement (!locked)

    }//end loop method
}
