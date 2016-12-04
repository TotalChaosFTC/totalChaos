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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class TankRobotOp extends OpMode {
    double armDelta = 0.01;

    boolean iSawDpadUpAlready = false;
    boolean iSawDpadDownAlready = false;
    boolean iSawDpadUpAlreadyArm = false;
    boolean iSawDpadDownAlreadyArm = false;
    boolean iSawDpadLeftAlreadyWinch = false;
    boolean iSawDpadRightAlreadyWinch = false;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftShooter;
    DcMotor rightShooter;
    DcMotor ballCollect;
    DcMotor vortexSpinner;
    Servo shotControl;
    CRServo pusherRight;
    CRServo pusherLeft;
    final static double FAST = 1.0;
    final static double MED_FAST = 0.75;
    final static double MEDIUM = 0.5;
    final static double SLOW = 0.25;
    double armMode = MEDIUM;
    double mode = FAST;
    double winchMode = FAST;
    double flapPosition = 1;

    public void init()
    {


        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        leftShooter = hardwareMap.dcMotor.get("ls");
        rightShooter = hardwareMap.dcMotor.get("rs");
        ballCollect = hardwareMap.dcMotor.get("bc");
        vortexSpinner = hardwareMap.dcMotor.get("vtx");
        shotControl = hardwareMap.servo.get("sc");
        pusherLeft = hardwareMap.crservo.get("left");
        pusherRight =  hardwareMap.crservo.get("right");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop()
    {
        // When dpad is pushed up increase one mode
        //When dpad is pushed down decrease by one mode
        if (gamepad1.dpad_up) {
            if(!iSawDpadUpAlready) {
                iSawDpadUpAlready = true;
                mode = mode + 0.25;
            }
        }
        else {
            iSawDpadUpAlready = false;
        }

        if (gamepad1.dpad_down) {
            if(!iSawDpadDownAlready) {
                iSawDpadDownAlready = true;
                mode = mode - 0.25;
            }
        }
        else {
            iSawDpadDownAlready = false;
        }
        mode = Range.clip(mode, 0.25, 0.75 );


        if (gamepad2.dpad_up) {
            if(!iSawDpadUpAlreadyArm) {
                iSawDpadUpAlreadyArm = true;
                armMode = armMode + 0.05;
            }
        }
        else {
            iSawDpadUpAlreadyArm = false;
        }

        if (gamepad2.dpad_down) {
            if(!iSawDpadDownAlreadyArm) {
                iSawDpadDownAlreadyArm = true;
                armMode = armMode - 0.05;
            }
        }
        else {
            iSawDpadDownAlreadyArm = false;
        }
        armMode = Range.clip(armMode, 0.1, 1 );

        if (gamepad1.right_trigger > 0){
            ballCollect.setPower(0.5);
        }
        else if (gamepad1.left_trigger > 0){
            ballCollect.setPower(-0.5);
        }
        else {
            ballCollect.setPower(0);
        }
        if (gamepad2.y){
           vortexSpinner.setPower(0.75);
        }
        else if (gamepad2.a){
            vortexSpinner.setPower(-0.75);
        }
        else {
            vortexSpinner.setPower(0);
        }

        if(gamepad2.left_bumper){
            flapPosition = flapPosition + armDelta;
            shotControl.setPosition(flapPosition);

        }

        if(gamepad2.right_bumper){
            flapPosition = flapPosition - armDelta;
            shotControl.setPosition(flapPosition);
        }
        if(gamepad2.x){
            flapPosition = 0.47;
            shotControl.setPosition(flapPosition);
            armMode = 0.65;
        }

        if(gamepad2.b){
            flapPosition = 0.42;
            shotControl.setPosition(flapPosition);
            armMode = 0.75;
        }
        telemetry.addData("Flap Position", "%f, BS Power %f", flapPosition, armMode);
        telemetry.update();

        double pushLeftPower = gamepad2.left_stick_y;
        double pushRightPower = gamepad2.right_stick_y;

        pusherLeft.setPower(-pushLeftPower);
        pusherRight.setPower(pushRightPower);

        double left = gamepad1.left_stick_y;
        double right= gamepad1.right_stick_y;

        if(gamepad2.right_trigger > 0){
            rightShooter.setPower(armMode);
            leftShooter.setPower(armMode);
        }
        else {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
        }

        right = (double)scaleInput(right);
        left =  (double)scaleInput(left);

        right= Range.clip(right, -mode, mode);
        left= Range.clip(left, -mode, mode);


        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);

    }


    @Override
    public void stop()
    {
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale * mode;
    }
}

