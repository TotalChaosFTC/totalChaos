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

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

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


public abstract class BackForward extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftShooter;
    DcMotor rightShooter;
    DcMotor ballCollect;
    DcMotor vortexSpinner;
    Servo shotControl;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double RED = 0;
    static final double BLUE = 0;
    static final double LEFT = 0;
    static final double RIGHT = 0;


    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private navXPIDController.PIDResult yawPIDResult;

    private boolean calibration_complete = false;

    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    long waitTime = 1000;

    public void initialize() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftFront = hardwareMap.dcMotor.get("motor_1");
        rightFront = hardwareMap.dcMotor.get("motor_2");
        leftBack = hardwareMap.dcMotor.get("motor_3");
        rightBack = hardwareMap.dcMotor.get("motor_4");
        leftShooter = hardwareMap.dcMotor.get("motor_5");
        rightShooter = hardwareMap.dcMotor.get("motor_6");
        ballCollect = hardwareMap.dcMotor.get("motor_7");
        shotControl = hardwareMap.servo.get("servo1");
        vortexSpinner = hardwareMap.dcMotor.get("motor_8");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset


        // Wait for the game to start (driver presses PLAY)
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);


        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.PITCH);


        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);



        waitForStart();

        /*while ( !calibration_complete ) {
            // navX-Micro Calibration completes automatically ~15 seconds after it is
            // powered on, as long as the device is still.  To handle the case where the
            // navX-Micro has not been able to calibrate successfully, hold off using
            // the navX-Micro Yaw value until calibration is complete.

            telemetry.addData("Step4a","");
            telemetry.update();

            calibration_complete = !navx_device.isCalibrating();

            telemetry.addData("Step4b","");
            telemetry.update();

            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        telemetry.addData("Step4c","");
        telemetry.update();

        navx_device.zeroYaw();
        telemetry.addData("Step5","");
        telemetry.update();
        */
        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 10.0;
        int DEVICE_TIMEOUT_MS = 500;
        yawPIDResult = new navXPIDController.PIDResult();


    }










    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderTurn(double leftTurn, double rightTurn, double leftAngle, double rightAngle) throws InterruptedException{




        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         int newLeftTurn;
            int newRightTurn;
            newLeftTurn = leftFront.getCurrentPosition() + (int)(leftAngle * COUNTS_PER_INCH);
            newRightTurn = rightFront.getCurrentPosition() + (int)(rightAngle * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTurn);
            leftBack.setTargetPosition(newLeftTurn);
            rightFront.setTargetPosition(newRightTurn);
            rightBack.setTargetPosition(newRightTurn);


            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setMotorPower(leftTurn, rightTurn);
            final double TOTAL_RUN_TIME_SECONDS = 30.0;
            int DEVICE_TIMEOUT_MS = 500;
            navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

            DecimalFormat df = new DecimalFormat("#.##");

        while (opModeIsActive() &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        robot.setMotorPower(leftTurn, rightTurn);

                    } else {
                        double output = yawPIDResult.getOutput();
                        robot.frontLeft.setPower(output);
                        robot.frontRight.setPower(-output);

                    }
                } else {
			     A timeout occurred

                }
            }

        robot.stopMotors();
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                */
    }




    public void encoderDrive(double leftSpeed, double rightSpeed,
                             double leftInches, double rightInches
    ) throws InterruptedException {

        int newLeftTarget;
        int newRightTarget;
        int DEVICE_TIMEOUT_MS = 500;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int)( leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            leftBack.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            rightBack.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            leftBack.setPower(leftSpeed);
            rightBack.setPower(rightSpeed);            // keep looping while we are still active, and there is time left, and both motors are running.

            /*while ( !calibration_complete ) {

                calibration_complete = !navx_device.isCalibrating();
                if (!calibration_complete) {
                    telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                }
            }
            */
            //navx_device.zeroYaw();
            yawPIDController.enable(true);
            while (opModeIsActive() &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                    if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                        if (yawPIDResult.isOnTarget()) {
                            leftFront.setPower(leftSpeed);
                            rightFront.setPower(rightSpeed);
                            leftBack.setPower(leftSpeed);
                            rightBack.setPower(rightSpeed);
                        } else {
                            double output = yawPIDResult.getOutput();

                            leftFront.setPower(leftSpeed+ output);
                            rightFront.setPower(rightSpeed- output);
                            leftBack.setPower(leftSpeed+ output);
                            rightBack.setPower(rightSpeed- output);
                            leftSpeed = leftSpeed + output;
                            rightSpeed = rightSpeed - output;

                        }

                    } else {
			        /* A timeout occurred */
                    }

                // Display it for the driver.
                double output = yawPIDResult.getOutput();

                telemetry.update();


                idle();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }



    public void colorSensorDrive(double color, double colorSpeed) throws InterruptedException {
        /*
        robot.setMotorPower(colorSpeed,colorSpeed);
        if (color == BLUE) {

            if (robot.beaconColorSensor.blue() > robot.beaconColorSensor.red()) {
                robot.stopMotors();
                robot.pusherRight.setPower(1);
                sleep(1000);
                robot.pusherRight.setPower(-1);
                sleep(1000);
                robot.pusherRight.setPower(0);

            }
            else if (robot.beaconColorSensor.red() > robot.beaconColorSensor.blue()) {
                    robot.stopMotors();
                    robot.pusherLeft.setPower(1);
                    sleep(1000);
                    robot.pusherLeft.setPower(-1);
                    sleep(1000);
                    robot.pusherLeft.setPower(0);

            }
            else {

            }

        }
        if (color == RED){
            if (robot.beaconColorSensor.red() > robot.beaconColorSensor.blue()) {
                robot.stopMotors();
                robot.pusherLeft.setPower(1);
                sleep(1000);
                robot.pusherLeft.setPower(-1);
                sleep(1000);
                robot.pusherLeft.setPower(0);

            }
            else if (robot.beaconColorSensor.blue() > robot.beaconColorSensor.red()) {
                robot.stopMotors();
                robot.pusherRight.setPower(1);
                sleep(1000);
                robot.pusherRight.setPower(-1);
                sleep(1000);
                robot.pusherRight.setPower(0);

            }
            else {
                telemetry.addData("LOL PROGRAMMING SUCKS YOU CAN'T EVEN GET A COLOR SENSOR TO WORK", "HAHAHAHA");

            }
         }
    */ }
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