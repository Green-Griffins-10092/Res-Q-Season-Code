package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David F. on 11/10/2015.
 * To store the hardware for the robot
 */
public final class RobotHardware {

    public static final String[] HARDWARE_MOTOR_NAMES = {"left drive 1", "left drive 2", "right drive 1", "right drive 2", "left arm pivot", "right arm pivot", "spool"};
    public static final String[] HARDWARE_SERVO_NAMES = {"left shifter servo", "right shifter servo", "ratchet servo"};
    public static final String[] HARDWARE_SENSOR_NAMES = {};

    //shifter variables
    private Shifter leftDrive;
    private Shifter rightDrive;

    //motor variables
    private DcMotor spoolMotor;
    private DcMotor leftArmPivot;
    private DcMotor rightArmPivot;

    //servo variables
    private Servo spoolLockServo;

    private RobotHardware()
    {}

    /**
     * This method creates and fills a RobotHardware object
     * This is the only way to create a RobotHardware object outside of RobotHardware
     * @param hardwareMap
     * A hardware map, which must contain the hardware names for the hardware
     * @return
     * A fully populated HardwareObject
     */
    public static RobotHardware initialize(HardwareMap hardwareMap) {
        RobotHardware hardware = new RobotHardware();  //initialize the object

        //assign all the fields to their values
        hardware.leftDrive = new Shifter(hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[0]), hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[1]),
                hardwareMap.servo.get(HARDWARE_SERVO_NAMES[0]));
        hardware.rightDrive = new Shifter(hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[2]), hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[3]),
                hardwareMap.servo.get(HARDWARE_SERVO_NAMES[1]));
        hardware.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        hardware.rightDrive.setDirection(DcMotor.Direction.REVERSE);

        hardware.leftArmPivot = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[4]);
        hardware.rightArmPivot = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[5]);
        hardware.leftArmPivot.setDirection(DcMotor.Direction.FORWARD);
        hardware.rightArmPivot.setDirection(DcMotor.Direction.REVERSE);

        hardware.spoolMotor = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[6]);
        hardware.spoolMotor.setDirection(DcMotor.Direction.FORWARD);

        hardware.spoolLockServo = hardwareMap.servo.get(HARDWARE_SERVO_NAMES[2]);

        return hardware; //return the object
    }
}
