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
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Vector;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
/*public class RedArmAutoOpLeague2 extends OpMode {
    double armDelta = 0.01;
    double armPosition = 0;
    int loopNumber = 0;
    boolean iSawDpadUpAlready = false;
    boolean iSawDpadDownAlready = false;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armTwist;
    DcMotor armLift;
    DcMotor frontSweeper;
    Servo ClickerRight;
    Servo ClickerLeft;
    Vector<Step> steps;
    Step currentStep;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    final static int MOVEARM = 4;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 2.75;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static int MOVE = 1;
    final static int RIGHT = 2;
    final static int LEFT = 3;
    public class Step {
        public double distance;
        public double leftPower;
        public double rightPower;
        public int rightCounts;
        public int leftCounts;
        public Step(double dist, double left, double right, int stepType) {
            distance = dist;
            if (stepType == MOVE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = right;
            }
            else {
                if (stepType == RIGHT) {
                    rightCounts = convertDistance(distance);
                    leftCounts = rightCounts;
                    leftPower = -left;
                    rightPower = right;
                }
                else{
                    rightCounts = convertDistance(distance);
                    leftCounts = rightCounts;
                    leftPower = left;
                    rightPower = -right;
                }
            }
        }
        public int convertDistance(double distance){
            double  rotations = distance / CIRCUMFERENCE;
            double counts = ENCODER_CPR * rotations * GEAR_RATIO;
            return (int) counts;
        }
    };

    public void init()
    {
        leftFront = hardwareMap.dcMotor.get("motor_1");
        rightFront = hardwareMap.dcMotor.get("motor_2");
        leftBack = hardwareMap.dcMotor.get("motor_3");
        rightBack = hardwareMap.dcMotor.get("motor_4");
        armTwist = hardwareMap.dcMotor.get("motor_5");
        armLift = hardwareMap.dcMotor.get("motor_6");
        frontSweeper = hardwareMap.dcMotor.get("motor_7");
        ClickerLeft = hardwareMap.servo.get("servo1");
        ClickerRight = hardwareMap.servo.get("servo2");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        steps = new Vector<Step>();
        steps.add(new Step(25, 0.5, 0.5, MOVE));
        steps.add(new Step(6, 0.5, 0.5, LEFT));
        steps.add(new Step(45, 0.5, 0.5, MOVE));
        steps.add(new Step(5.5, 0.5, 0.5, LEFT));
        steps.add(new Step(50, 0.5, 0.5, MOVE));
        steps.add(new Step(48, 0.5,0.5, LEFT));
        currentStep = steps.get(0);
        currentStepIndex = 0;
    }

    @Override
    public void loop(){
        if (state == ATREST){
            resetEncoders();
            state = WAITFORRESETENCODERS;
        }
        else if( state == WAITFORRESETENCODERS) {
            if (areEncodersReset()){
                setMotorPower(currentStep.leftPower, currentStep.rightPower);
                frontSweeper.setPower(0.75);
                state = WAITFORCOUNTS;
            }
        }
        else if (state== WAITFORCOUNTS){
            if (areCountsReached(currentStep.leftCounts, currentStep.rightCounts)){
                setMotorPower(0, 0);
                frontSweeper.setPower(0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()){
                    state = MOVEARM;
                    armTwist.setPower(0.2);
                }
                else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }

            }
            else if( state == MOVEARM){
                loopNumber++;
                if (loopNumber > 100){
                    armTwist.setPower(0);
                    state = FINISHED;
                }
            }

        }
    }

    public void resetEncoders() {
        leftFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    public boolean areEncodersReset() {
            return leftFront.getCurrentPosition() == 0 &&
                    rightFront.getCurrentPosition() == 0 &&
                    leftBack.getCurrentPosition() == 0 &&
                    rightBack.getCurrentPosition() == 0;
        }

    public boolean areCountsReached(int leftCounts, int rightCounts) {
        return ( Math.abs(leftFront.getCurrentPosition()) >= Math.abs(leftCounts) &&
                Math.abs(rightFront.getCurrentPosition()) >= Math.abs(rightCounts) &&
                Math.abs(leftBack.getCurrentPosition())>= Math.abs(leftCounts) &&
                Math.abs(rightBack.getCurrentPosition()) >= Math.abs(rightCounts) );

    }

    public void setMotorPower(double leftPower,double rightPower){
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
    }
}


*/