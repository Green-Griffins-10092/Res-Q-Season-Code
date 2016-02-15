package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

/**
 * Created by David on 12/12/2015.
 * The autonomous for the blue alliance.
 */
public class BlueAuto extends RampAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        blueSide = true;
        super.runOpMode();
    }
}