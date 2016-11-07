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

/*public class LeagueTwoRedOp extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armTwist;
    DcMotor armLift;
    DcMotor frontSweeper;

    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 2.75;

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
        leftFront = hardwareMap.dcMotor.get("motor_1");
        rightFront = hardwareMap.dcMotor.get("motor_2");
        leftBack = hardwareMap.dcMotor.get("motor_3");
        rightBack = hardwareMap.dcMotor.get("motor_4");
        armTwist = hardwareMap.dcMotor.get("motor_5");
        armLift = hardwareMap.dcMotor.get("motor_6");
        frontSweeper = hardwareMap.dcMotor.get("motor_7");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        // while (getRuntime() < endTime) {
            frontSweeper.setPower(-.75);
            move(25, -1.0, -1.0);
            turn(6, -0.5, -0.5);
            move(50, -1.0, -1.0);
            turn(12, -0.5, -0.5);
            move(50, -0.5, -0.5);
            frontSweeper.setPower(0);
        //}

      //  leftMotor.setPowerFloat();
       // rightMotor.setPowerFloat();
        setMotorPower(0.0, 0.0);

    }
  }
    public void resetEncoders() {
        leftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    public void waitForResetEncoders() {
        while( leftFront.getCurrentPosition() != 0 ||
                rightFront.getCurrentPosition() != 0 ||
                leftBack.getCurrentPosition() != 0 ||
                rightBack.getCurrentPosition() != 0 ) {
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void waitForCounts(int leftCounts, int rightCounts) {
        while( Math.abs(leftFront.getCurrentPosition()) < Math.abs(leftCounts) ||
                Math.abs(rightFront.getCurrentPosition()) < Math.abs(rightCounts) ||
                Math.abs(leftBack.getCurrentPosition())< Math.abs(leftCounts) ||
                Math.abs(rightBack.getCurrentPosition()) < Math.abs(rightCounts) ) {
            try {
                waitForNextHardwareCycle();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    public void turn(double distance , double leftPower , double rightPower) {
        resetEncoders();
        waitForResetEncoders();
        int counts = convertDistance(distance);
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        setMotorPower(leftPower, -rightPower);
        waitForCounts(-counts, counts);
        setMotorPower(0.0, 0.0);
        try {
            waitForNextHardwareCycle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void move(double distance , double leftPower , double rightPower) {
        resetEncoders();
        waitForResetEncoders();
        int counts = convertDistance(distance);
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        setMotorPower(leftPower, rightPower);
        waitForCounts(counts, counts);
        setMotorPower(0.0, 0.0);
        try {
            waitForNextHardwareCycle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public int convertDistance(double distance){
        double  rotations = distance / CIRCUMFERENCE;
        double counts = ENCODER_CPR * rotations * GEAR_RATIO;
        return (int) counts;
    }

    public void setMotorPower(double rightPower,double leftPower){
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
    }
}
*/