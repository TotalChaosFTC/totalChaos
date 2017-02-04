package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class MechBot
{
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft    = null;
    public DcMotor  backRight   = null;
    public DcMotor  leftShooter = null;
    public DcMotor  rightShooter = null;
    public CRServo  pusherLeft   = null;
    public CRServo  pusherRight  = null;
    public ColorSensor beaconColorSensor = null;
    public TouchSensor rightFrontTouchSensor = null;
    public TouchSensor rightBackTouchSensor = null;
    public TouchSensor leftFrontTouchSensor = null;
    public TouchSensor leftBackTouchSensor = null;
    public Servo popper = null;
    public DcMotor  sweeper  = null;
    public ColorSensor bottomColorSensor;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;


    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public MechBot(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.dcMotor.get("lf");
        frontRight = hwMap.dcMotor.get("rf");
        backLeft = hwMap.dcMotor.get("lb");
        backRight = hwMap.dcMotor.get("rb");
        leftShooter = hwMap.dcMotor.get("ls");
        rightShooter = hwMap.dcMotor.get("rs");
        sweeper = hwMap.dcMotor.get("swp");
        popper = hwMap.servo.get("pop");
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        pusherLeft = hwMap.crservo.get("left");
        pusherRight = hwMap.crservo.get("right");
        pusherLeft.setPower(0);
        pusherRight.setPower(0);
        beaconColorSensor = hwMap.colorSensor.get("beacon");
        leftFrontTouchSensor = hwMap.touchSensor.get("lFront");
        leftBackTouchSensor = hwMap.touchSensor.get("lBack");
        rightFrontTouchSensor = hwMap.touchSensor.get("rFront");
        rightFrontTouchSensor = hwMap.touchSensor.get("rBack");

        //bottomColorSensor = hwMap.colorSensor.get("bottom");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconColorSensor.enableLed(false);
        //bottomColorSensor.enableLed(true);


    }
    public void ResetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setMotorPowerInternal (double leftPower, double rightPower){
        backLeft.setPower(leftPower);
        frontLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);

    }

    public void setMotorPower (double leftPower, double rightPower){
        int counter = 0;
        boolean nullFlag = true;
        while (counter < 3 && nullFlag) {
            try {
                setMotorPowerInternal(leftPower, rightPower);
                nullFlag = false;
            } catch (NullPointerException e) {
                counter++;
            }
        }
    }
    public void setMechleftInternal (double leftPower, double rightPower){
        backLeft.setPower(-leftPower);
        frontLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(-rightPower);

    }

    public void setMechleft (double leftPower, double rightPower){
        int counter = 0;
        boolean nullFlag = true;
        while (counter < 3 && nullFlag) {
            try {
                setMechleftInternal(leftPower, rightPower);
                nullFlag = false;
            } catch (NullPointerException e) {
                counter++;
            }
        }
    }
    public void setMechrightInternal (double leftPower, double rightPower){
        backLeft.setPower(leftPower);
        frontLeft.setPower(-leftPower);
        backRight.setPower(-rightPower);
        frontRight.setPower(rightPower);

    }

    public void setMechright (double leftPower, double rightPower){
        int counter = 0;
        boolean nullFlag = true;
        while (counter < 3 && nullFlag) {
            try {
                setMechrightInternal(leftPower, rightPower);
                nullFlag = false;
            } catch (NullPointerException e) {
                counter++;
            }
        }
    }
    public void stopMotors() {
        setMotorPower(0,0);
    }


}

