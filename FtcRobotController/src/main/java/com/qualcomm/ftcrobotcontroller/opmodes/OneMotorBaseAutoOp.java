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
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Vector;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
/*public abstract class OneMotorBaseAutoOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor frontSweeper;
    Servo ClimberDrop;
    TouchSensor touch1;
    Vector<Step> steps;
    Step currentStep;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 5;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static int FRONTRIGHT = 1;
    final static int BACKRIGHT = 2;
    final static int FRONTLEFT = 3;
    final static int BACKLEFT = 4;
    final static int WAITFORTOUCH = 5;
    final static int MOVEARM = 6;
    final static boolean FORWARD = true;
    final static boolean BACKWARD = false;

    public class Step {
        public double distance;
        public double frontLeftPower;
        public double frontRightPower;
        public int rightCounts;
        public int leftCounts;
        public boolean sweeperDirection;
        public double armPosition;
        public double backRightPower;
        public double backLeftPower;
        public int sType;
        public Step(double dist, double backRight, double backLeft, double frontRight, double frontLeft, int stepType, boolean direction) {
            distance = dist;
            sType = stepType;
            if (stepType == BACKRIGHT){
                rightCounts = convertDistance(distance);
                backRightPower = backRight;
            }
            else if (stepType == FRONTRIGHT) {
                rightCounts = convertDistance(distance);
                backLeftPower = backLeft;
            }
            else if(stepType == BACKLEFT){
                leftCounts = convertDistance(distance);
                backLeftPower = backLeft;
            }

            else if(stepType == FRONTLEFT){
                leftCounts = convertDistance(distance);
                frontLeftPower = frontLeft;
            }

            else if (stepType == WAITFORTOUCH){
                /*setMotorPower(currentStep.leftPower, currentStep.rightPower);
                if (touch1.getValue() == 1){
                    setMotorPower(0, 0);
                }*/
/*
            }

            else{
                armPosition = dist;
            }
            sweeperDirection = direction;
        }

        public int convertDistance(double distance){
            double  rotations = distance / CIRCUMFERENCE;
            double counts = ENCODER_CPR * rotations * GEAR_RATIO;
            return (int) counts;
        }
    }

    public void init()
    {
        leftFront = hardwareMap.dcMotor.get("motor_1");
        rightFront = hardwareMap.dcMotor.get("motor_2");
        leftBack = hardwareMap.dcMotor.get("motor_3");
        rightBack = hardwareMap.dcMotor.get("motor_4");
        frontSweeper = hardwareMap.dcMotor.get("motor_7");
        ClimberDrop = hardwareMap.servo.get("servo5");
        touch1 = hardwareMap.touchSensor.get("touch_1");
        ClimberDrop.setPosition(0.71);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        ClimberDrop.setDirection(Servo.Direction.REVERSE);
        steps = new Vector<Step>();
        initSteps();
        currentStep = steps.get(0);
        currentStepIndex = 0;
    }

    public abstract void initSteps();
    @Override
    public void loop(){
        if (state == ATREST){
            if(currentStep.sType == MOVEARM){
                ClimberDrop.setPosition(currentStep.distance);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }
            else {
                resetEncoders();
                state = WAITFORRESETENCODERS;
            }
        }
        else if( state == WAITFORRESETENCODERS) {
            if (areEncodersReset()){
                setMotorPower(currentStep.backLeftPower, currentStep.frontLeftPower, currentStep.backRightPower, currentStep.frontRightPower);
                if (currentStep.sweeperDirection == FORWARD) {
                    frontSweeper.setPower(0.75);
                }
                else if(currentStep.sweeperDirection == BACKWARD){
                    frontSweeper.setPower(-0.75);
                }
                state = WAITFORCOUNTS;
            }
        }
        else if (state== WAITFORCOUNTS) {
            if (areCountsReached(currentStep.leftCounts, currentStep.rightCounts)) {
                setMotorPower(0,0,0,0);
                frontSweeper.setPower(0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }
        }
    }

    @Override
    public void stop(){
        state = ATREST;
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

    public void setMotorPower(double backLeftPower,double frontLeftPower, double backRightPower, double frontRightPower){
        leftFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightBack.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }
}

*/
