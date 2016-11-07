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
import com.qualcomm.robotcore.hardware.ColorSensor;
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
/*public abstract class NewBaseAutoOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armTwist;
    DcMotor armLift;
    DcMotor frontSweeper;
    DcMotor winch;
    Servo clickerRight;
    Servo clickerLeft;
    Servo buttonFlap;
    Servo sensorSlide;
    Servo ClimberDrop;
    ColorSensor color1;
    Vector<Step> steps;
    Step currentStep;
    int counts = 0;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    final static int WAITFORCOLOR = 4;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 2.75;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static int MOVE = 1;
    final static int RIGHT = 2;
    final static int LEFT = 3;
    final static int MOVEARM = 4;
    final static int WAITFORTOUCH = 5;
    final static int BACK = 6;
    final static int BLUE = 7;
    final static int RED = 8;
    final static int WAIT = 9;
    final static int MOVEPUSHER = 10;
    final static int FORWARD = 11;
    final static int BACKWARD = 12;
    final static int NONE = 13;

    public class Step {
        public double distance;
        public double leftPower;
        public double rightPower;
        public int rightCounts;
        public int leftCounts;
        public int sweeperDirection;
        public double armPosition;
        public int sType;
        public Step(double dist, double left, double right, int stepType, int direction) {
            distance = dist;
            sType = stepType;
            if (stepType == MOVE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = right;
            }
            else if (stepType == RIGHT) {
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = right;
            }
            else if(stepType == LEFT){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = -right;
            }
            else if(stepType == BACK){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == BLUE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == RED){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if (stepType == WAITFORTOUCH){
                /*setMotorPower(currentStep.leftPower, currentStep.rightPower);
                if (touch1.getValue() == 1){
                    setMotorPower(0, 0);
                }
            }
            else if(stepType == WAIT){
                leftPower = 0;
                rightPower = 0;
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
        armTwist = hardwareMap.dcMotor.get("motor_5");
        armLift = hardwareMap.dcMotor.get("motor_6");
        frontSweeper = hardwareMap.dcMotor.get("motor_7");
        clickerLeft = hardwareMap.servo.get("servo1");
        clickerRight = hardwareMap.servo.get("servo2");
        buttonFlap = hardwareMap.servo.get("servo3");
        sensorSlide = hardwareMap.servo.get("servo4");
        ClimberDrop = hardwareMap.servo.get("servo5");
        winch = hardwareMap.dcMotor.get("motor_8");
        color1 = hardwareMap.colorSensor.get("color1");
        ClimberDrop.setPosition(0.6);
        clickerRight.setPosition(0);
        clickerLeft.setPosition(1);
        sensorSlide.setPosition(0.5);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        clickerLeft.setDirection(Servo.Direction.REVERSE);
        clickerRight.setDirection(Servo.Direction.REVERSE);
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
            else if(currentStep.sType == MOVEPUSHER){
                buttonFlap.setPosition(currentStep.distance);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;

                }
            }
            else if(currentStep.sType == BLUE|| currentStep.sType == RED) {
                state = WAITFORCOLOR;
            }
            else if(currentStep.sType == WAIT){
                frontSweeper.setPower(0);
                if (counts == 400){
                    currentStepIndex = currentStepIndex + 1;
                    counts = 0;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;

                    }
                }
                else{
                    counts = counts+1;                }
            }
            else {
                resetEncoders();
                state = WAITFORRESETENCODERS;
            }
        }
        else if (state == WAITFORCOLOR) {
            if (currentStep.sType == BLUE) {
                telemetry.addData("Looking for Blue", color1.blue());
                if (color1.blue() > 0.99) {
                    sensorSlide.setPosition(0.5);
                    currentStepIndex = currentStepIndex + 1;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;
                    }
                }
                else {
                    sensorSlide.setPosition(0);
                }
            } else if (currentStep.sType == RED) {
                telemetry.addData("Looking for Red", color1.red());
                if (color1.red() > 0.99) {
                    sensorSlide.setPosition(0.5);
                    currentStepIndex = currentStepIndex + 1;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;
                    }
                }
                else{
                    sensorSlide.setPosition(0);
                }
            }
        }
        else if( state == WAITFORRESETENCODERS) {
            if (areEncodersReset()){
                    setMotorPower(currentStep.leftPower, currentStep.rightPower);
                    if (currentStep.sweeperDirection == FORWARD) {
                        frontSweeper.setPower(1);
                    } else if (currentStep.sweeperDirection == BACKWARD) {
                        frontSweeper.setPower(-1);
                    }
                    else if (currentStep.sweeperDirection == NONE) {
                        frontSweeper.setPower(0);
                    }
                    state = WAITFORCOUNTS;
            }
        }
        else if (state== WAITFORCOUNTS) {
            telemetry.addData("sweeper power", frontSweeper.getPower());
            if (areCountsReached(currentStep.leftCounts, currentStep.rightCounts)) {
                setMotorPower(0, 0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }
        }
        else if (state == FINISHED){
            frontSweeper.setPower(0);

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
