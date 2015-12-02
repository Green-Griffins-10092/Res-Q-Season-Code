package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by David on 12/1/2015.
 * This is the master Teleop program for the FIRST Res-Q season.
 * See Gamepad map.png for controls.
 */
public class Teleop extends OpMode{

    private RobotHardware hardware;

    @Override
    public void init() {
        hardware = RobotHardware.initialize(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
