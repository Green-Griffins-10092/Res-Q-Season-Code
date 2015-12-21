package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 12/19/2015.
 *
 * This class will store the hardware for the robot
 */
public class RobotHardware {

    public static final String[] MOTOR_NAMES = {};

    private DcMotor leftArmPivot, rightArmPivot;
    private DcMotor leftDriveMotor, rightDriveMotor;
    private DcMotor armTelescopeMotor1, armTelescopeMotor2;
    private DcMotor armIntakeMotor;
    private DcMotor turretPivotMotor;

    private GyroSensor robotRotationGyro;

    public RobotHardware(HardwareMap hardwareMap)
    {

    }
}