package com.qualcomm.ftcrobotcontroller.opmodes.RobotItems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David on 11/4/2015.
 *
 * holds information for a shifter
 */
public class Shifter {
    private DcMotor motor1;
    private DcMotor motor2;
    private Servo shifter;

    private ShifterPosition position;

    final double ENGAGE_DRIVE = 0;
    final double NEUTRAL = .5;
    final double ENGAGE_ARM = 1;

    public void shift(ShifterPosition position){
        this.position = position;
        switch (position){
            case DRIVE:
                shifter.setPosition(ENGAGE_DRIVE);
                break;
            case NEUTRAL:
                shifter.setPosition(NEUTRAL);
                break;
            case ARM:
                shifter.setPosition(ENGAGE_ARM);
                break;
        }
    }

    public ShifterPosition getPosition(){
        return position;
    }

    /*
     *  the commands to interact with the motors
     */
    public void setDirection(DcMotor.Direction direction) {
        motor1.setDirection(direction);
        motor2.setDirection(direction);
    }

    public DcMotor.Direction getDirection(){
        return motor1.getDirection();
    }

    public void setPower(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public double getPower(){
        return motor1.getPower();
    }

    public boolean isBusy()
    {
        return motor1.isBusy() && motor2.isBusy();
    }

    public void setPowerFloat(){
        motor1.setPowerFloat();
        motor2.setPowerFloat();
    }

    public boolean getPowerFloat(){
        return motor1.getPowerFloat() && motor2.getPowerFloat();
    }

    public void setTargetPosition(int position){
        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);
    }

    public int getTargetPosition(){
        return motor1.getTargetPosition();
    }

    public int getCurrentPosition(){
        return (motor1.getCurrentPosition()+motor2.getTargetPosition())/2;
    }

    public void setChannelMode(DcMotorController.RunMode mode){
        motor1.setChannelMode(mode);
        motor2.setChannelMode(mode);
    }

    public DcMotorController.RunMode getChannelMode(){
        return motor1.getChannelMode();
    }

    /*
     *  end motor commands
     */

    public static enum ShifterPosition{
        DRIVE,
        NEUTRAL,
        ARM;

        ShifterPosition(){
        }
    }
}