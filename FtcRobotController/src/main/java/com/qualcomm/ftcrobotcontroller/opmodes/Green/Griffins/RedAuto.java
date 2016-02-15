package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

/**
 * Created by David on 12/12/2015.
 * The autonomous for the red alliance.
 */
public class RedAuto extends RampAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        blueSide = false;
        super.runOpMode();
    }
}