package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class RoverBot
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
    public TouchSensor beaconTouchSensor = null;
    public Servo shotControl = null;
    public DcMotor  ballCollect  = null;
    public DcMotor vortexSpinner = null;
    //public ColorSensor bottomColorSensor;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;


    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public RoverBot(){

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
        ballCollect = hwMap.dcMotor.get("bc");
        shotControl = hwMap.servo.get("sc");
        vortexSpinner = hwMap.dcMotor.get("vtx");
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
        beaconTouchSensor = hwMap.touchSensor.get("touch");
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
    public void runUsingEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runToPosition(double inches, double COUNTS_PER_INCH){
        int newLeftTarget = backLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newRightTarget = backRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(newLeftTarget);
        backLeft.setTargetPosition(newLeftTarget);
        frontRight.setTargetPosition(newRightTarget);
        backRight.setTargetPosition(newRightTarget);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setMotorPower (double leftPower, double rightPower){
            backLeft.setPower(leftPower);
            frontLeft.setPower(leftPower);
            backRight.setPower(rightPower);
            frontRight.setPower(rightPower);

    }

    public void setMotorPowerx (double leftPower, double rightPower){
        int counter = 0;
        boolean nullFlag = true;
        while (counter < 3 && nullFlag) {
            try {
                //setMotorPowerInternal(leftPower, rightPower);
                nullFlag = false;
            } catch (NullPointerException e) {
                counter++;
            }
        }
    }
    public void setMechleft (double power){
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(-power);
    }
    public void setMechRight (double power){
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);
        frontRight.setPower(power);
    }
    public void stopMotors() {
        setMotorPower(0,0);
    }


}

