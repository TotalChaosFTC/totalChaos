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

package org.firstinspires.ftc.robotcontroller.internal;

import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftcrobotcontroller.opmodes.MechWheelsOp;
import com.qualcomm.ftcrobotcontroller.opmodes.TankRobotOp;
import com.qualcomm.ftcrobotcontroller.opmodes.TankRoverOp;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

import org.firstinspires.ftc.robotcontroller.external.samples.AutoBlue1;
import org.firstinspires.ftc.robotcontroller.external.samples.AutoBlue2;
import org.firstinspires.ftc.robotcontroller.external.samples.AutoRed1;
import org.firstinspires.ftc.robotcontroller.external.samples.AutoRed2;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
import org.firstinspires.ftc.robotcontroller.external.samples.CornerTKO;
import org.firstinspires.ftc.robotcontroller.external.samples.JabbyRed;
import org.firstinspires.ftc.robotcontroller.external.samples.PunchRed;
import org.firstinspires.ftc.robotcontroller.external.samples.TKOred;

/**
 * {@link FtcOpModeRegister} is responsible for registering opmodes for use in an FTC game.
 * @see #register(OpModeManager)
 */
public class FtcOpModeRegister implements OpModeRegister {

    /**
     * {@link #register(OpModeManager)} is called by the SDK game in order to register
     * OpMode classes or instances that will participate in an FTC game.
     *
     * There are two mechanisms by which an OpMode may be registered.
     *
     *  1) The preferred method is by means of class annotations in the OpMode itself.
     *  See, for example the class annotations in {@link ConceptNullOp}.
     *
     *  2) The other, retired,  method is to modify this {@link #register(OpModeManager)}
     *  method to include explicit calls to OpModeManager.register().
     *  This method of modifying this file directly is discouraged, as it
     *  makes updates to the SDK harder to integrate into your code.
     *
     * @param manager the object which contains methods for carrying out OpMode registrations
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
     * @see com.qualcomm.robotcore.eventloop.opmode.Autonomous
     */
    public void register(OpModeManager manager) {

        /**
         * Register OpModes implemented in the Blocks visual programming language.
         */
        BlocksOpMode.registerAll(manager);

        /**
         * Register OpModes that use the annotation-based registration mechanism.
         */
        AnnotatedOpModeRegistrar.register(manager);

        /**
         * Any manual OpMode class registrations should go here.
         */
    manager.register("IceTeleOp", TankRobotOp.class);
    //manager.register("ServoTestOp", ServoTestOp.class);
    //manager.register("TankServoOp", TankServoOp.class);
    //manager.register("TestEncoderOp", TestEncoder.class);
    //manager.register("LinearTestOp", LinearTestOp.class);
    //manager.register("PulleyArmOp", PulleyArmOp.class);
    //manager.register("ContinuousServoOp", ContinuousServoOp.class);
    manager.register("MechWheels", MechWheelsOp.class);
    manager.register("TKOred" , TKOred.class);
    manager.register("CornerTKO" , CornerTKO.class);
        manager.register("PunchRed" , PunchRed.class);
        manager.register("JabbyRed" , JabbyRed.class);
    //manager.register("AutoTestOp", AutoTestOp.class);
    //manager.register("FourMotorMeasureEncoder", FourMotorMeasureEncoder.class);
    //manager.register("League0MountainOp", League0AutoOp.class);
    //manager.register("FifteenMinuteOp", FifteenMinuteRun.class);
    //manager.register("League0FloorGoalOp", League0FloorGoalOp.class);
    //manager.register("AutoOp", AutoOp.class);
    //manager.register("LeagueTwoOp", LeagueTwoOp.class);
    //manager.register("MechWheelsAuto", AutoMechRed1.class);
    //manager.register("LeagueTwoRedOp", MechTestOp.class);
    //manager.register("MechWheels", MechWheelsOp.class);
    manager.register("BlueNormal1", AutoBlue1.class);
    manager.register("ShootBlueAuto", AutoBlue2.class);
    manager.register("RedNormal1", AutoRed1.class);
    manager.register("ShootRedAuto", AutoRed2.class);
    //manager.register("ColorSensorTest", ColorSensorTestOp.class);
    //manager.register(“BeaconPractice", BeaconPracticeOp.class);
    //manager.register("RedMoveBack", RedAutoOpLeague2MoveBack.class);
    //manager.register("CornerRed", CornerRedAutoOpLeague2.class);
    //manager.register("CornerBlue", CornerAutoOpLeague2.class);
    //manager.register("ColorSensorTest", ColorSensorOp.class);
    manager.register("BlehBot :(", TankRoverOp.class);
    //manager.register("TouchSensorTest", TouchSensorTest.class);
    //manager.register("ArmBlue", ArmAutoOpLeague2.class);
    //manager.register("ArmRed", RedArmAutoOpLeague2.class);
    }
}
