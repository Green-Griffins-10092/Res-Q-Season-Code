package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

/**
 * Created by David on 2/19/2016.
 * The red auto for starting by the tape and dumping the climbers in the bucket.
 */
public class RedClimberAuto extends ClimberAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        blueSide = false;
        waitInMilliseconds = 0;
        super.runOpMode();
    }
}
