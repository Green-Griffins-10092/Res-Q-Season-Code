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

        for (DcMotor motor :
                hardwareMap.dcMotor) {
            motor.setPowerFloat();
        }

        //servos do not have a float method
    }

    @Override
    public void loop() {
        //TODO: have a joystick control direction of servo
        //TODO: have a button that switches the direction of the servo and motor
        //TODO: have controls that precisely change the servo value (motor too?)
        //TODO: reset encoder value button? then also a button that moves motor very precisely, and changes motor mode

        if (testMotor != null){
            testMotor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Motor power", testMotor.getPower());
            telemetry.addData("Motor encoder position", testMotor.getCurrentPosition());
            telemetry.addData("Motor direction", testMotor.getDirection());
        }

        if (testServo != null) {
            testServo.setPosition(.33);
            telemetry.addData("Servo position", testServo.getPosition());
            telemetry.addData("Servo direction", testServo.getDirection());
        }
    }

}
