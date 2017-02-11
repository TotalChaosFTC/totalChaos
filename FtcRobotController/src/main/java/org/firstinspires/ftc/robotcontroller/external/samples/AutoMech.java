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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file illusttes the concept of driving a path based on encoder counts.
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

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled
public abstract class AutoMech extends LinearOpMode {

    /* Declare OpMode members. */
    MechBot robot = new MechBot();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    //Andy Mark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 3.0 / (43.0 / 16.0);     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.7 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final int RED = 0;
    static final int BLUE = 1;

    static final int RIGHT = 0;
    static final int LEFT = 1;


    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;


    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    long waitTime = 1000;

    public void initialize() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;


        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderTurn(double power,  double angle) throws InterruptedException{
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();


        int newLeftTurn;
        int newRightTurn;
        double radius = 8.25;
        double inches = angle * 2 * Math.PI * radius / 360;
        newLeftTurn = robot.frontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightTurn = robot.frontRight.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        robot.frontLeft.setTargetPosition(newLeftTurn);
        robot.backLeft.setTargetPosition(newLeftTurn);
        robot.frontRight.setTargetPosition(newRightTurn);
        robot.backRight.setTargetPosition(newRightTurn);

        telemetry.addData("Target turn", newRightTurn);
        telemetry.update();

        double leftPower = (angle > 0.0) ? power : -1.0*power;
        double rightPower = (angle < 0.0) ? power : -1.0*power;

        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.setMotorPower(leftPower, rightPower);
        while (opModeIsActive() &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
            idle();
        }
        telemetry.addData("Turn", "1");
        telemetry.update();


        robot.stopMotors();
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

    }

    public void encoderDrive(double power, double inches) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int DEVICE_TIMEOUT_MS = 500;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            double leftSpeed = power;
            double rightSpeed = power;
            robot.setMotorPower(leftSpeed, rightSpeed);
            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
                idle();
            }
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.stopMotors();
            idle();
        }
    }
    public void encoderLeft (double power, double inches) throws InterruptedException {

        int newNegativeTarget;
        int newPostiveTarget;
        int DEVICE_TIMEOUT_MS = 500;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newNegativeTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newNegativeTarget = newNegativeTarget * -1;
            newPostiveTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newNegativeTarget);
            robot.backLeft.setTargetPosition(newPostiveTarget);
            robot.frontRight.setTargetPosition(newPostiveTarget);
            robot.backRight.setTargetPosition(newNegativeTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(-power);
            while (robot.frontRight.isBusy() && robot.frontLeft.isBusy()) {
                idle();
            }
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.stopMotors();
            idle();
        }
    }
    public void encoderDiagonalLeft (double power, double inches) throws InterruptedException {

        int newTarget;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setTargetPosition(newTarget);
            robot.frontRight.setTargetPosition(newTarget);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(0);
            while (robot.backLeft.isBusy() && robot.frontRight.isBusy()) {
                idle();
            }
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.stopMotors();
            idle();
        }
    }
    public void encoderDiagonalRight (double power, double inches) throws InterruptedException {

        int newTarget;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newTarget);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setPower(power);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(power);
            while (robot.frontLeft.isBusy() && robot.backRight.isBusy()) {
                idle();
            }
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.stopMotors();
            idle();
        }
    }
    public void encoderRight (double power, double inches) throws InterruptedException {

        int newNegativeTarget;
        int newPostiveTarget;
        int DEVICE_TIMEOUT_MS = 500;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newNegativeTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newNegativeTarget = newNegativeTarget * -1;
            newPostiveTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newPostiveTarget);
            robot.backLeft.setTargetPosition(newNegativeTarget);
            robot.frontRight.setTargetPosition(newNegativeTarget);
            robot.backRight.setTargetPosition(newPostiveTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setPower(-power);
            robot.backLeft.setPower(power);
            robot.frontRight.setPower(power);
            robot.backRight.setPower(-power);
            while (robot.frontRight.isBusy() && robot.frontLeft.isBusy()) {
                idle();
            }
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.stopMotors();
            idle();
        }
    }

        public void touchSensorDrive(double direction, double power, double inches) throws InterruptedException {
            int newNegativeTarget;
            int newPostiveTarget;
            int DEVICE_TIMEOUT_MS = 500;

            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();

            if (direction  == LEFT) {
                newNegativeTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newNegativeTarget = newNegativeTarget * -1;
                newPostiveTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                robot.frontLeft.setTargetPosition(newNegativeTarget);
                robot.backLeft.setTargetPosition(newPostiveTarget);
                robot.frontRight.setTargetPosition(newPostiveTarget);
                robot.backRight.setTargetPosition(newNegativeTarget);


                // Turn On RUN_TO_POSITION
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.setMechleft(power,power);
                while (robot.backLeft.isBusy() && robot.backRight.isBusy() && (!robot.leftFrontTouchSensor.isPressed() || !robot.leftBackTouchSensor.isPressed()) ){
                    idle();
                }
            }
            else if (direction == RIGHT){
                newNegativeTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newNegativeTarget = newNegativeTarget * -1;
                newPostiveTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                robot.frontLeft.setTargetPosition(newPostiveTarget);
                robot.backLeft.setTargetPosition(newNegativeTarget);
                robot.frontRight.setTargetPosition(newNegativeTarget);
                robot.backRight.setTargetPosition(newPostiveTarget);


                // Turn On RUN_TO_POSITION
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.setMechright(power,power);
                while (robot.backLeft.isBusy() && robot.backRight.isBusy() && (!robot.rightFrontTouchSensor.isPressed() || !robot.rightBackTouchSensor.isPressed()) ){
                    idle();
                }
            }

            robot.stopMotors();
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            idle();
        }



    public void stoponBeaconColor (double power, double inches, int color) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        newLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightTarget = robot.frontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        robot.frontLeft.setTargetPosition(newLeftTarget);
        robot.backLeft.setTargetPosition(newLeftTarget);
        robot.frontRight.setTargetPosition(newRightTarget);
        robot.backRight.setTargetPosition(newRightTarget);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setMotorPower(power,power);

        if (color == RED) {
            while ((robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                if (robot.beaconColorSensor.red() >= 4 )
                    break;
                idle();
            }
        }

        if (color == BLUE) {
            while ((robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                if (robot.beaconColorSensor.blue() >= 4  )
                    break;
                idle();
            }
        }
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopMotors();
        idle();
    }
    public void colorSensorDrive(int color) throws InterruptedException {

        if (color == BLUE) {

            if (robot.beaconColorSensor.blue() > robot.beaconColorSensor.red()) {
                robot.pusherRight.setPower(-1);
                sleep(1000);
                robot.pusherRight.setPower(1);
                sleep(1000);
                robot.pusherRight.setPower(0);
                 telemetry.addData("YAY!","IT FOUND BLUE!");

            }
            else if (robot.beaconColorSensor.red() > robot.beaconColorSensor.blue()) {
                robot.pusherLeft.setPower(-1);
                sleep(1000);
                robot.pusherLeft.setPower(1);
                sleep(1000);
                robot.pusherLeft.setPower(0);
                telemetry.addData("YAY!","IT FOUND RED!");
                telemetry.update();

            }
            else {

            }

        }
        else if (color == RED){
            if (robot.beaconColorSensor.red() > robot.beaconColorSensor.blue()) {
                robot.pusherLeft.setPower(-1);
                sleep(1000);
                robot.pusherLeft.setPower(1);
                sleep(1000);
                robot.pusherLeft.setPower(0);
                telemetry.addData("YAY!","IT FOUND RED!");
                telemetry.update();

            }
            else if (robot.beaconColorSensor.blue() > robot.beaconColorSensor.red()) {
                robot.pusherRight.setPower(-1);
                sleep(1000);
                robot.pusherRight.setPower(1);
                sleep(1000);
                robot.pusherRight.setPower(0);
                telemetry.addData("YAY!","IT FOUND BLUE!");
                telemetry.update();

            }
            else {
                telemetry.addData("The color sensor is not getting any readings", "");

            }
        }
    }
}
    /*public void lineFollow (double side, double followSpeed) throws InterruptedException {
     if (side == LEFT) {
         if (robot.bottomColorSensor.argb() == 16) {
             if (reachedTheBeacon()) {
                 robot.stopMotors();
             } else {
                 robot.setMotorPower(0.5, 0.75);
                 sleep(1000);
             }

         }
         if (robot.bottomColorSensor.argb() == 0) {
             if (reachedTheBeacon()) {

                 robot.stopMotors();

             } else {
                 robot.setMotorPower(0.5, 0.75);
                 sleep(1000);
             }

         }
     }
     if (side == RIGHT){
         if (robot.bottomColorSensor.argb() == 16) {
             if (reachedTheBeacon()) {
                 robot.stopMotors();
             } else {
                 robot.setMotorPower(0.75, 0.5);
                 sleep(1000);

             }

         }
         if (robot.bottomColorSensor.argb() == 0) {
             if (reachedTheBeacon()) {

                 robot.stopMotors();

             } else {
                 robot.setMotorPower(0.75, 0.5);
                 sleep(1000);
             }

         }
     }

    }
    public boolean reachedTheBeacon(){
        if (robot.beaconColorSensor.blue() < 2 && robot.beaconColorSensor.red() < 2){
            return false;
        }
        else {
            return true;
        }
    }


}
*/