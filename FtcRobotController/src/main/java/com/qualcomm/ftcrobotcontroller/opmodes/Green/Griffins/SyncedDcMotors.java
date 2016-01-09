package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController.RunMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 1/1/2016.
 * To control multiple motors at the same time
 */
public class SyncedDcMotors {

    //constants
    public static final int ALL_SAME = 0;
    public static final int ALTERNATING = 1;

    //variables
    DcMotor[] motors;
    int directionPattern;

    //note it is illegal to pass zero motors names
    public SyncedDcMotors(HardwareMap hardwareMap, DcMotor.Direction direction, int directionPattern, String... motorName) {
        if (motorName.length == 0) {
            throw new IllegalArgumentException("can not take 0 motors");
        }

        this.directionPattern = directionPattern;
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
        for (int i = 0; i < motors.length; i++) {
            if (directionPattern == ALL_SAME) {
                motors[i].setDirection(direction);
            } else if (directionPattern == ALTERNATING) {
                if (i % 2 == 0) {
                    motors[i].setDirection(direction);
                } else {
                    if (direction == DcMotor.Direction.FORWARD) {
                        motors[i].setDirection(DcMotor.Direction.REVERSE);
                    } else {
                        motors[i].setDirection(DcMotor.Direction.FORWARD);
                    }
                }
            }
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