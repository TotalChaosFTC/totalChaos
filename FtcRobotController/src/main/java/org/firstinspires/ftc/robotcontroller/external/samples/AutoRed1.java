/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


public class AutoRed1 extends AutoBasewNavx {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        encoderDrive(0.35, 28);
        robot.shotControl.setPosition(0.42);
        robot.leftShooter.setPower(0.15);
        robot.rightShooter.setPower(0.10);
        sleep(500);
        robot.leftShooter.setPower(0.35);
        robot.rightShooter.setPower(0.30);
        sleep(500);
        robot.leftShooter.setPower(0.55);
        robot.rightShooter.setPower(0.50);
        sleep(500);
        robot.leftShooter.setPower(0.75);
        robot.rightShooter.setPower(0.70);
        sleep(750);
        robot.ballCollect.setPower(0.5);
        sleep(1500);
        robot.shotControl.setPosition(0.42);
        robot.leftShooter.setPower(0.75);
        robot.rightShooter.setPower(0.70);
        robot.ballCollect.setPower(0);
        sleep(500);
        robot.ballCollect.setPower(0.5);
        sleep(1500);
        robot.leftShooter.setPower(0.55);
        robot.rightShooter.setPower(0.50);
        sleep(500);
        robot.leftShooter.setPower(0.35);
        robot.rightShooter.setPower(0.30);
        sleep(500);
        robot.ballCollect.setPower(0);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        sleep(1000);
        encoderTurn(-0.15, -50);
        encoderDrive(0.35, 40);
        encoderTurn(0.15, -37.75);
        touchSensorDrive(0.15);
        colorSensorDrive(RED);
        //encoderDrive(-0.2, -12);
        //encoderTurn(0.2, 90);
        //encoderDrive(0.2, 47.5);
        //encoderTurn(0.2, -90);
        //touchSensorDrive(0.2);
        //colorSensorDrive(RED);
        //encoderTurn(0.2, -135);
        //encoderDrive(0.2, 20);

        
    }
}