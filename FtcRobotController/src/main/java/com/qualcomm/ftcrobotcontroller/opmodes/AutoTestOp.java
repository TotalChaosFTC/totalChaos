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
import com.qualcomm.robotcore.hardware.DcMotorController;

/*
 * Linear Tele Op Mode
 * <p>
 * Enables control of the robot via the gamepad.
 * NOTE: This op mode will not work with the NXT Motor Controllers. Use an Nxt op mode instead.
 */

/*public class AutoTestOp extends LinearOpMode {
  DcMotor motorRight;
  DcMotor motorLeft;
  DcMotor motorRight2;
  DcMotor motorLeft2;
  DcMotor motorRight3;
  DcMotor motorLeft3;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 100;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    // Servo arm;
  // double armDelta = 0.01;
  // double armPosition;

    @Override
    public void runOpMode() throws InterruptedException {
    {
        // arm = hardwareMap.servo.get("ServoArm");
       // armPosition = 0;
        // arm.setPosition(0);
        motorRight = hardwareMap.dcMotor.get("motor_1");
        motorLeft = hardwareMap.dcMotor.get("motor_2");
        motorRight2 = hardwareMap.dcMotor.get("motor_3");
        motorLeft2 = hardwareMap.dcMotor.get("motor_4");
        motorRight3 = hardwareMap.dcMotor.get("motor_5");
        motorLeft3 = hardwareMap.dcMotor.get("motor_6");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorLeft3.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        double currentTime = getRuntime();
        double endTime = currentTime + 50000;
        while (getRuntime() < endTime) {
            move(4.0);
            turn(4.0);
            move(4.0);
            turn(4.0);

        }

      //  leftMotor.setPowerFloat();
       // rightMotor.setPowerFloat();
        setMotorPower(0.0 ,0.0);

    }
  }
    public void resetEncoders() {
        motorLeft.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public boolean waitForResetEncoders() {
        return motorLeft.getCurrentPosition() == 0 &&
                motorRight.getCurrentPosition() == 0;
    }

    public void turn(double distance) {
        resetEncoders();
        while( !waitForResetEncoders() ) {
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        double  ROTATIONS = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
        motorLeft.setTargetPosition((int)counts);
        motorRight.setTargetPosition((int) -counts);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft2.setTargetPosition((int) counts);
        motorRight2.setTargetPosition((int) -counts);
        motorLeft2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft3.setTargetPosition((int)counts);
        motorRight3.setTargetPosition((int) -counts);
        motorLeft3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setMotorPower(1.0, -1.0);
    }
    public void move(double distance) {
        resetEncoders();
        while( !waitForResetEncoders() ) {
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        double  ROTATIONS = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * ROTATIONS * GEAR_RATIO;
        motorLeft.setTargetPosition((int)counts);
        motorRight.setTargetPosition((int) -counts);
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft2.setTargetPosition((int) counts);
        motorRight2.setTargetPosition((int) -counts);
        motorLeft2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight2.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorLeft3.setTargetPosition((int)counts);
        motorRight3.setTargetPosition((int) -counts);
        motorLeft3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight3.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        setMotorPower(1.0, 1.0);
    }
    public void setMotorPower(double rightPower,double leftPower){
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
        motorLeft2.setPower(leftPower);
        motorRight2.setPower(rightPower);
        motorLeft3.setPower(leftPower);
        motorRight3.setPower(rightPower);
    }
}
*/