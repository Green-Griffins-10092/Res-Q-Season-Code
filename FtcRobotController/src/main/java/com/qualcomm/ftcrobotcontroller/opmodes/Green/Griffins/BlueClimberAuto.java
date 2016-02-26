package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

/**
 * Created by David on 2/26/2016.
 * The Auto for the blue side dumping the climbers.
 */
public class BlueClimberAuto extends ClimberAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        blueSide = true;
        waitInMilliseconds = 0;
        super.runOpMode();
    }
}
