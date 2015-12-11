package com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by David F. on 11/10/2015.
 * To store the hardware for the robot
 */
public final class RobotHardware {

    public static final String[] HARDWARE_MOTOR_NAMES = {"left drive 1", "left drive 2", "right drive 1", "right drive 2", "left arm pivot", "right arm pivot", "spool", "intake"};
    public static final String[] HARDWARE_SERVO_NAMES = {"left shifter servo", "right shifter servo", "hanging servo", "bucket", "bacon pusher"};
    public static final String[] HARDWARE_SENSOR_NAMES = {"gyro"};

    //shifter variables
    private Shifter leftDrive;
    private Shifter rightDrive;

    //motor variables
    private DcMotor spoolMotor;
    private DcMotor leftArmPivot;
    private DcMotor rightArmPivot;
    private DcMotor armIntakeMotor;


    //servo variables
    private Servo armLockServo;
    private Servo bucketDumpServo;
    private Servo beaconButtonPusher; //not used yet
    private Servo[] sledServos; //not used yet

    //sensor variables (eventually)
    private ModernRoboticsI2cGyro gyro;

    //other variables to keep track of things
    BucketPosition bucketPosition;

    private RobotHardware() {
    }

    /**
     * This method creates and fills a RobotHardware object
     * This is the only way to create a RobotHardware object outside of RobotHardware
     *
     * @param hardwareMap A hardware map, which must contain the hardware names for the hardware
     * @return A fully populated HardwareObject
     */
    public static RobotHardware initialize(HardwareMap hardwareMap) {
        RobotHardware hardware = new RobotHardware();  //initialize the object

        //assign all the fields to their values

        //shifters first
        hardware.leftDrive = new Shifter(hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[0]), hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[1]),
                hardwareMap.servo.get(HARDWARE_SERVO_NAMES[0]));
        hardware.rightDrive = new Shifter(hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[2]), hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[3]),
                hardwareMap.servo.get(HARDWARE_SERVO_NAMES[1]));
        hardware.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        hardware.rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //motors second
        hardware.leftArmPivot = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[4]);
        hardware.rightArmPivot = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[5]);
        hardware.leftArmPivot.setDirection(DcMotor.Direction.FORWARD);
        hardware.rightArmPivot.setDirection(DcMotor.Direction.REVERSE);

        hardware.spoolMotor = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[6]);
        hardware.spoolMotor.setDirection(DcMotor.Direction.FORWARD);

        hardware.armIntakeMotor = hardwareMap.dcMotor.get(HARDWARE_MOTOR_NAMES[7]);
        hardware.armIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        //servos third
        hardware.armLockServo = hardwareMap.servo.get(HARDWARE_SERVO_NAMES[2]);
        hardware.bucketDumpServo = hardwareMap.servo.get(HARDWARE_SERVO_NAMES[3]);
        hardware.bucketDumpServo.setDirection(Servo.Direction.FORWARD);
        hardware.setBucketPosition(BucketPosition.CENTER);
        //hardware.beaconButtonPusher = hardwareMap.servo.get(HARDWARE_SERVO_NAMES[4]);

        //sensors fourth
        try {
            hardware.gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get(HARDWARE_SENSOR_NAMES[0]);
            hardware.gyro.calibrate();
        } catch (IllegalArgumentException e) {
            //sensors do not exist
        }

        return hardware; //return the object
    }

    public Shifter getLeftDrive() {
        return leftDrive;
    }

    public Shifter getRightDrive() {
        return rightDrive;
    }

    //care must be taken to synchronize the spool motor and
    //the left and right shifters when the shifters are in ARM position
    public DcMotor getSpoolMotor() {
        return spoolMotor;
    }

    public DcMotor getArmIntakeMotor() {
        return armIntakeMotor;
    }

    public void engageArmLock(boolean engage){
        //TODO: engage the arm lock if engage is true
    }

    public void setArmPivotPower(double power){
        leftArmPivot.setPower(power);
        rightArmPivot.setPower(power);
    }

    public double getArmPivotPower()
    {
        return leftArmPivot.getPower();
    }

    public BucketPosition getBucketPosition() {
        return bucketPosition;
    }

    public void setBucketPosition(BucketPosition bucketPosition) {
        this.bucketPosition = bucketPosition;
        switch (bucketPosition){
            case LEFT:
                bucketDumpServo.setPosition(.55);
                break;
            case CENTER:
                bucketDumpServo.setPosition(.442);
                break;
            case RIGHT:
                bucketDumpServo.setPosition(.374);
                break;
        }
    }

    public enum BucketPosition{
        LEFT,
        RIGHT,
        CENTER
    }
}
