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

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public class TankRoverOp extends OpMode {
    boolean iSawDpadUpAlready = false;
    boolean iSawDpadDownAlready = false;
    boolean iSawDpadUpAlreadyArm = false;
    boolean iSawDpadDownAlreadyArm = false;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor vortex;
    DcMotor rightBack;
    final static double FAST = 1.0;
    final static double MED_FAST = 0.75;
    final static double MEDIUM = 0.5;
    final static double SLOW = 0.25;
    double armMode = FAST;
    double mode = FAST;
    CRServo pusherLeft;
    CRServo pusherRight;

    double pusherposition = 0.5;
    ColorSensor beaconColorSensor;
    ExtraColorSensor bottomColorSensor;






    public void init()
    {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        vortex = hardwareMap.dcMotor.get("vtx");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        beaconColorSensor = hardwareMap.colorSensor.get("beacon");
        //bottomColorSensor = hardwareMap.colorSensor.get("bottom");
        //bottomColorSensor = new ExtraColorSensor(hardwareMap.deviceInterfaceModule.get("dim"), 3);
        pusherLeft = hardwareMap.crservo.get("pusherLeft");
        pusherRight = hardwareMap.crservo.get("pusherRight");

        pusherLeft.setPower(0);
        pusherRight.setPower(0);
    }

    @Override
    public void loop()
    {
        telemetry.addData("Red", beaconColorSensor.red());
        telemetry.update();

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
        mode = Range.clip(mode, 0.25, 1 );

        // when leftstick is pushed up move forward
        //when rightstick is pushed down move backwards
        double left = -gamepad1.left_stick_y;
        double right= -gamepad1.right_stick_y;



        right = (double)scaleInput(right);
        left =  (double)scaleInput(left);

        right= Range.clip(right, -mode, mode);
        left= Range.clip(left, -mode, mode);

        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);


        if (gamepad1.y){
            vortex.setPower(50);
        }
        else {
            vortex.setPower(0);
        }

        if (gamepad1.a){
            vortex.setPower(-50);
        }
        else{
            vortex.setPower(0);
        }
        if (gamepad2.x){
            pusherposition = 1;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherLeft.setPower(1);
        }
        else{
            pusherposition = 0 ;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherLeft.setPower(0);
        }
        if (gamepad2.b) {
            pusherposition = -1;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherLeft.setPower(-1);
        }
        else{
            pusherposition = 0;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherLeft.setPower(0);
        }

        if (gamepad2.y){
            pusherposition = 1;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherRight.setPower(1);
        }
        else{
            pusherposition = 0 ;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherRight.setPower(0);
        }
        if (gamepad2.a) {
            pusherposition = -1;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherRight.setPower(-1);
        }
        else{
            pusherposition = 0;
            pusherposition = Range.clip(pusherposition, -1, 1);
            pusherRight.setPower(0);
        }

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
