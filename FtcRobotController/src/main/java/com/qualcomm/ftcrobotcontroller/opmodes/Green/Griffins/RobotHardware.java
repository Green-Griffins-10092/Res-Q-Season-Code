package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David on 12/19/2015.
 * <p/>
 * This class will store the hardware for the robot
 */
public class RobotHardware {

    public static final String[] MOTOR_NAMES = {"pivot 1", "pivot 2", "extend 1", "extend 2",
                                                "drive left", "drive right", "intake", "turret"};
    public static final String[] SERVO_NAMES = {"leftPanelServo", "rightPanelServo"};
    public static final String[] SENSOR_NAMES = {"gyro"};

    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 = 1680;
    public static final int ENCODER_COUNTS_PER_ROTATION_NEVEREST_40 = 1120;

    public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATIONS =  54 / 16.0; // 16 on the little, 54 teeth on the big
    public static final double ENCODER_COUNTS_PER_TURRET_DEGREES = ENCODER_COUNTS_PER_ROTATION_NEVEREST_60 * MOTOR_ROTATIONS_PER_TURRET_ROTATIONS / 360;
    public static final double MOTOR_ROTATIONS_PER_ARM_TELESCOPE_ROTATIONS = 2;
    public static final double ENCODER_COUNTS_PER_ARM_INCHES = 123;

    private SyncedDcMotors armPivotMotors;
    private SyncedDcMotors armTelescopeMotors;
    private DcMotor leftDriveMotor, rightDriveMotor;
    private DcMotor armIntakeMotor;
    private DcMotor turretPivotMotor;
    private Servo leftPanelServo;
    private Servo rightPanelServo;

    private ModernRoboticsI2cGyro robotRotationGyro;

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
        armTelescopeMotors = new SyncedDcMotors(hardwareMap, DcMotor.Direction.REVERSE, SyncedDcMotors.ALTERNATING, MOTOR_NAMES[2], MOTOR_NAMES[3]);

        //setting up the arm intake motor
        armIntakeMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[6]);
        armIntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //setting up the turret pivot motor
        turretPivotMotor = hardwareMap.dcMotor.get(MOTOR_NAMES[7]);
        turretPivotMotor.setDirection(DcMotor.Direction.FORWARD);

        // right panel has a down position of 1 and a up position of .55
        // left panel has a down position of .9 and a up position of .39 (when reversed)

        //setting up the panel servos
        leftPanelServo = hardwareMap.servo.get(SERVO_NAMES[0]);
        leftPanelServo.setDirection(Servo.Direction.FORWARD);
        leftPanelServo.scaleRange(.1, .65);
        rightPanelServo = hardwareMap.servo.get(SERVO_NAMES[1]);
        rightPanelServo.setDirection(Servo.Direction.FORWARD);
        rightPanelServo.scaleRange(.55, 1);

        //setting up the gyro sensor
        robotRotationGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get(SENSOR_NAMES[0]);
    }

    public ModernRoboticsI2cGyro getRobotRotationGyro() {
        return robotRotationGyro;
    }

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

    public SyncedDcMotors getArmTelescopeMotors() {return armTelescopeMotors; }

    /*
     * The position is given with 0 being down and 1 being released.
     */
    public void setPanelPosition(double position) {
        leftPanelServo.setPosition(position);
        rightPanelServo.setPosition(1 - position);
    }
}