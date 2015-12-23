package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 12/19/2015.
 *
 * This class will store the hardware for the robot
 */
public class RobotHardware {

    public static final String[] MOTOR_NAMES = {"left arm", "right arm", "left drive", "right drive",
                                                "telescope 1", "telescope 2", "intake", "turret"};
    public static final String[] SENSOR_NAMES = {"turning gyro"};

    private DcMotor leftArmPivot, rightArmPivot;
    private DcMotor leftDriveMotor, rightDriveMotor;
    private DcMotor armTelescopeMotor1, armTelescopeMotor2;
    private DcMotor armIntakeMotor;
    private DcMotor turretPivotMotor;

    private ModernRoboticsI2cGyro robotRotationGyro;

    public RobotHardware(HardwareMap hardwareMap)
    {
        //setting up arm pivot motors
        leftArmPivot = hardwareMap.dcMotor.get(MOTOR_NAMES[0]);
        rightArmPivot = hardwareMap.dcMotor.get(MOTOR_NAMES[1]);
        leftArmPivot.setDirection(DcMotor.Direction.FORWARD);
        rightArmPivot.setDirection(DcMotor.Direction.REVERSE);

        //setting up drive motors
        leftDriveMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[2]);
        rightDriveMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[3]);
        leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        //setting up the telescope motors
        armTelescopeMotor1 = hardwareMap.dcMotor.get(MOTOR_NAMES[4]);
        armTelescopeMotor2 = hardwareMap.dcMotor.get(MOTOR_NAMES[5]);
        armTelescopeMotor1.setDirection(DcMotor.Direction.FORWARD);
        armTelescopeMotor2.setDirection(DcMotor.Direction.FORWARD);

        //setting up the arm intake motor
        armIntakeMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[6]);
        armIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the turret pivot motor
        turretPivotMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[7]);
        turretPivotMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the gyro sensor
        robotRotationGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get(SENSOR_NAMES[0]);
    }
}