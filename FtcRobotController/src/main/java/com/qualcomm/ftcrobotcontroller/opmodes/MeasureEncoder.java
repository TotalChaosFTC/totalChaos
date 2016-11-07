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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
/*public class MeasureEncoder extends OpMode {
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorRight2;
    DcMotor motorLeft2;
    DcMotor motorRight3;
    DcMotor motorLeft3;

    boolean aButtonPressed = false;
    boolean bButtonPressed  = false;
    boolean xButtonPressed  = false;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 100;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double  ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
    @Override

    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_1");
        motorLeft = hardwareMap.dcMotor.get("motor_2");
        motorRight2 = hardwareMap.dcMotor.get("motor_3");
        motorLeft2 = hardwareMap.dcMotor.get("motor_4");
        motorRight3 = hardwareMap.dcMotor.get("motor_5");
        motorLeft3 = hardwareMap.dcMotor.get("motor_6");

        //motorRight.setDirection(DcMotor.Direction.REVERSE);


        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorLeft3.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start ()
    {
    }

    @Override
    public void loop()
    {
        if (gamepad1.a) {
            resetEncoders();
            aButtonPressed = true;
        }
        if( aButtonPressed ) {
            if( waitForResetEncoders() ) {
                turn(200);
                aButtonPressed = false;
            }
        }
        if (gamepad1.b) {
            resetEncoders();
            bButtonPressed = true;
        }
        if( bButtonPressed ) {
            if( waitForResetEncoders() ) {
                turn(100);
                bButtonPressed = false;
            }
        }
        if (gamepad1.x) {
            resetEncoders();
            xButtonPressed = true;
        }
        if (xButtonPressed) {
            if( waitForResetEncoders() ) {
                move(100);
                xButtonPressed = false;
            }
        }

            telemetry.addData("Motor Target", COUNTS);
            telemetry.addData("left motors position", motorLeft.getCurrentPosition());
            telemetry.addData("right motors position",motorRight.getCurrentPosition() );
    }

    public void resetEncoders() {
        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public boolean waitForResetEncoders() {
        return motorLeft.getCurrentPosition() == 0 &&
                motorRight.getCurrentPosition() == 0;
    }

    public void turn(int counts) {
        motorLeft.setTargetPosition((int)counts);
        motorRight.setTargetPosition((int) -counts);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft2.setTargetPosition((int) counts);
        motorRight2.setTargetPosition((int) -counts);
        motorLeft2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft3.setTargetPosition((int) counts);
        motorRight3.setTargetPosition((int) -counts);
        motorLeft3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setMotorPower(1, -1);
    }
    public void move(int counts) {
        motorLeft.setTargetPosition((int)counts);
        motorRight.setTargetPosition((int) -counts);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft2.setTargetPosition((int) counts);
        motorRight2.setTargetPosition((int) -counts);
        motorLeft2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft3.setTargetPosition((int) counts);
        motorRight3.setTargetPosition((int) -counts);
        motorLeft3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setMotorPower(1, 1);
    }

    public void setMotorPower(double rightPower,double leftPower){
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
        motorLeft2.setPower(leftPower);
        motorRight2.setPower(rightPower);
        motorLeft3.setPower(leftPower);
        motorRight3.setPower(rightPower);
    }

    @Override
    public void stop()
    {
    }
}

*/