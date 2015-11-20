package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David on 11/19/2015.
 * To test servos and motors
 */
public class MotorServoTest extends OpMode{

    DcMotor testMotor;
    Servo testServo;

    @Override
    public void init() {
        try {
            testMotor = hardwareMap.dcMotor.get("test motor");
        } catch (IllegalArgumentException e) {
            telemetry.addData("Warning", "No test motor!");
        }

        try {
            testServo = hardwareMap.servo.get("test servo");
        } catch (Exception e) {
            telemetry.addData("Warning", "No test motor!");
        }
    }

    @Override
    public void loop() {

    }
}
