package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 1/1/2016.
 * To control multiple motors at the same time
 */
public class SyncedDcMotors {

    //variables
    DcMotor[] motors;


    //note it is illegal to pass zero motors names
    public SyncedDcMotors(HardwareMap hardwareMap, DcMotor.Direction direction, String... motorName) {
        if (motorName.length == 0) {
            throw new IllegalArgumentException("can not take 0 motors");
        }

        motors = new DcMotor[motorName.length];

        for (int i = 0; i < motorName.length; i++) {
            motors[i] = hardwareMap.dcMotor.get(motorName[i]);
        }

        setDirection(direction);
    }

    public DcMotor.Direction getDirection() {
        return motors[0].getDirection();
    }

    public void setDirection(DcMotor.Direction direction) {
        // TODO: 1/1/2016 allow for patterns?
        for (DcMotor motor : motors) {
            motor.setDirection(direction);
        }
    }

    public double getPower() {
        return motors[0].getPower();
    }

    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public boolean isBusy() {
        boolean status = false;

        for (DcMotor motor : motors) {
            status = status || motor.isBusy();
        }

        return status;
    }

    public void setPowerFloat() {
        for (DcMotor motor : motors) {
            motor.setPowerFloat();
        }
    }

    public boolean getPowerFloat() {
        return motors[0].getPowerFloat();
    }

    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    public int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public RunMode getMode() {
        return motors[0].getMode();
    }

    public void setMode(RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    /**
     * @deprecated
     */
    @Deprecated
    public RunMode getChannelMode() {
        return this.getMode();
    }

    /**
     * @deprecated
     */
    @Deprecated
    public void setChannelMode(RunMode mode) {
        this.setMode(mode);
    }
}