package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by David on 12/19/2015.
 * <p/>
 * This class will store the hardware for the robot
 */
public class RobotHardware {

    public static final String[] MOTOR_NAMES = {"pivot 1", "pivot 2", "extend 1", "extend 2",
                                                "drive left", "drive right", "intake", "turret"};
//    public static final String[] SENSOR_NAMES = {"turning gyro"};

    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 = 1680;
    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_40 = 1120;

    public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS = 6;
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREES = ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 * MOTOR_ROTATIONS_PER_TURRET_ROTATIONS / 360;
    public static final double MOTOR_ROTATIONS_PER_ARM_TELESCOPE_ROTATIONS = 2;

    private SyncedDcMotors armPivotMotors;
    private SyncedDcMotors armTelescopeMotors;
    private DcMotor leftDriveMotor, rightDriveMotor;
    private DcMotor armIntakeMotor;
    private DcMotor turretPivotMotor;

//    private ModernRoboticsI2cGyro robotRotationGyro;

    public RobotHardware(HardwareMap hardwareMap) {
        this.initialize(hardwareMap);
    }

    public void initialize(HardwareMap hardwareMap){
        //setting up arm pivot motors
        armPivotMotors = new SyncedDcMotors(hardwareMap, DcMotor.Direction.REVERSE, SyncedDcMotors.ALL_SAME, MOTOR_NAMES[0], MOTOR_NAMES[1]);

        //setting up drive motors
        leftDriveMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[4]);
        rightDriveMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[5]);
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the telescope motors
        armTelescopeMotors = new SyncedDcMotors(hardwareMap, DcMotor.Direction.FORWARD, SyncedDcMotors.ALTERNATING, MOTOR_NAMES[2], MOTOR_NAMES[3]);

        //setting up the arm intake motor
        armIntakeMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[6]);
        armIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the turret pivot motor
        turretPivotMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[7]);
        turretPivotMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the gyro sensor
//        robotRotationGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get(SENSOR_NAMES[0]);
    }

    /*public ModernRoboticsI2cGyro getRobotRotationGyro() {
        return robotRotationGyro;
    }*/

    public DcMotor getArmIntakeMotor() {
        return armIntakeMotor;
    }

    public DcMotor getTurretPivotMotor() {
        return turretPivotMotor;
    }

    public DcMotor getLeftDriveMotor() {
        return leftDriveMotor;
    }

    public DcMotor getRightDriveMotor() {
        return rightDriveMotor;
    }

    public SyncedDcMotors getArmPivotMotors() {
        return armPivotMotors;
    }

    public SyncedDcMotors getArmTelescopeMotors() {
        return armTelescopeMotors;
    }
}