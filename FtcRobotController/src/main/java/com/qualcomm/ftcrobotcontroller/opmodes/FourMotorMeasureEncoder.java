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
/*public class FourMotorMeasureEncoder extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;

    boolean aButtonPressed = false;
    boolean bButtonPressed  = false;
    boolean xButtonPressed  = false;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 2.75;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    @Override

    public void init() {
        leftFront = hardwareMap.dcMotor.get("motor_1");
        rightFront = hardwareMap.dcMotor.get("motor_2");
        leftBack = hardwareMap.dcMotor.get("motor_3");
        rightBack = hardwareMap.dcMotor.get("motor_4");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
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
                turn(12.4);
                aButtonPressed = false;
            }
        }
        if (gamepad1.b) {
            resetEncoders();
            bButtonPressed = true;
        }
        if( bButtonPressed ) {
            if( waitForResetEncoders() ) {
                turn(12.5);
                bButtonPressed = false;
            }
        }
        if (gamepad1.x && !xButtonPressed) {
            resetEncoders();
            xButtonPressed = true;
        }
        if (xButtonPressed) {
            if( waitForResetEncoders() ) {
                turn(12.6);
                xButtonPressed = false;
            }
        }

            telemetry.addData("left motors position", leftFront.getCurrentPosition());
            telemetry.addData("right motors position",rightFront.getCurrentPosition() );
    }

    public void resetEncoders() {
        leftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public boolean waitForResetEncoders() {
        return leftFront.getCurrentPosition() == 0 &&
                rightFront.getCurrentPosition() == 0 &&
                leftBack.getCurrentPosition() == 0 &&
                rightBack.getCurrentPosition() == 0;
    }

    public void move(double distance) {
        int counts = convertDistance(distance);
        leftFront.setTargetPosition((int)counts);
        rightFront.setTargetPosition((int) counts);
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftBack.setTargetPosition((int) counts);
        rightBack.setTargetPosition((int) counts);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        setMotorPower(1, -1);
    }
    public void turn(double distance) {
        int counts = convertDistance(distance);
        leftFront.setTargetPosition((int)-counts);
        rightFront.setTargetPosition((int) counts);
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftBack.setTargetPosition((int) -counts);
        rightBack.setTargetPosition((int) counts);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setMotorPower(1, 1);
    }

    public void setMotorPower(double rightPower,double leftPower){
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
    }

    public int convertDistance(double distance){
        double  rotations = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * rotations * GEAR_RATIO;
        return (int) counts;
    }

    @Override
    public void stop()
    {
    }
}

*/