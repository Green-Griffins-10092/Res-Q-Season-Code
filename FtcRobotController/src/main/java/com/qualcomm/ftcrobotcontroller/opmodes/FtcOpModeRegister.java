/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.BlueClimberAuto;
import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.BlueRampAuto;
import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.MotorServoTest;
import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.RedClimberAuto;
import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.RedRampAuto;
import com.qualcomm.ftcrobotcontroller.opmodes.Green.Griffins.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * Register Op Modes
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * The Op Mode Manager will call this method when it wants a list of all
     * available op modes. Add your op mode to the list to enable it.
     *
     * @param manager op mode manager
     */
    public void register(OpModeManager manager) {

        /*
         * register your op modes here.
         * The first parameter is the name of the op mode
         * The second parameter is the op mode class property
         *
         * If two or more op modes are registered with the same name, the app will display an error.
         */

        manager.register("NullOp", NullOp.class);

        /*
         * the motor servo test op mode
         */
        manager.register("Tester op mode", MotorServoTest.class);

        /*
         * the teleop
         */
        manager.register("Teleop", Teleop.class);

        /*
         * the autonomous programs
         */
        manager.register("A-0-Red-Ramp", RedRampAuto.class);
        manager.register("A-0-Blue-Ramp", BlueRampAuto.class);
        manager.register("A-0-Red-Climber", RedClimberAuto.class);
        manager.register("A-0-Blue-Climber", BlueClimberAuto.class);

        /*
         * the gyro testing functions
         */
//        manager.register("Gyro", MRGyroTest.class);
    }
}
