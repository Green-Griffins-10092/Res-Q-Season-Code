/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor
 * is configured with a name of "gyro".
 *
 *
 *
 */
public class MRGyroTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        GyroSensor sensorGyro;
        int xVal, yVal, zVal = 0;
        int heading = 0;

        DcMotor leftDrive, rightDrive;

        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();

        // get a reference to our GyroSensor object.
        sensorGyro = hardwareMap.gyroSensor.get("gyro");

        leftDrive = hardwareMap.dcMotor.get("left drive");
        rightDrive = hardwareMap.dcMotor.get("right drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // calibrate the gyro.
        sensorGyro.calibrate();

        // wait for the start button to be pressed.
        waitForStart();

        // make sure the gyro is calibrated.
        while (sensorGyro.isCalibrating()) {
            Thread.sleep(50);
        }

        //correcting algorithm
        double target = 90;
        while (sensorGyro.getHeading() < target-1 || sensorGyro.getHeading() > target + 1){
            double power = (target - sensorGyro.getHeading())/200;

            power = Range.clip(power, .05, .25);

            leftDrive.setPower(power);
            rightDrive.setPower(-power);

            telemetry.addData("1. p", power);
            telemetry.addData("4. h", String.format("%03d", sensorGyro.getHeading()));
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sensorGyro.resetZAxisIntegrator();
        //wait for advance signal
        while (!gamepad1.a)
        {
            waitForNextHardwareCycle();
        }

        // make sure the gyro is calibrated.
        while (sensorGyro.isCalibrating()) {
            Thread.sleep(50);
        }

        //hit the target first time around
        target = 180;
        while (sensorGyro.getHeading() < target-target/45 && opModeIsActive()) {
            double power = (target - sensorGyro.getHeading())/target;
            power = Range.clip(power, .01, .25);

            leftDrive.setPower(power);
            rightDrive.setPower(-power);

            telemetry.addData("1. p", power);
            telemetry.addData("4. h", String.format("%03d", sensorGyro.getHeading()));
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        while (opModeIsActive()) {

            leftDrive.setPower(-gamepad1.left_stick_y);
            rightDrive.setPower(-gamepad1.right_stick_y);

            // if the A and B buttons are pressed, reset Z heading.
            if (gamepad1.a && gamepad1.b) {
                // reset heading.
                sensorGyro.resetZAxisIntegrator();
            }

            /*// use the X and Y buttons to switch the mode
            if (gamepad1.x){
                sensorGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
            }else if (gamepad1.y){
                sensorGyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);
            }*/

            // get the x, y, and z values (rate of change of angle).
            xVal = sensorGyro.rawX();
            yVal = sensorGyro.rawY();
            zVal = sensorGyro.rawZ();

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            heading = sensorGyro.getHeading();

            telemetry.addData("1. x", String.format("%03d", xVal));
            telemetry.addData("2. y", String.format("%03d", yVal));
            telemetry.addData("3. z", String.format("%03d", zVal));
            telemetry.addData("4. h", String.format("%03d", heading));

            Thread.sleep(100);
        }
    }
}
