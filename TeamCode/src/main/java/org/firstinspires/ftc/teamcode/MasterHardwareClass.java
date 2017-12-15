package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//This class defines all the specific hardware for our robot.

public class MasterHardwareClass {
    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor verticalArmMotor = null;
    public CRServo clawServo = null;
    public Servo gemServo;


    /* Give place holder values for the motors and the grabber servo */
    double FrontLeftPower = 0;
    double FrontRightPower = 0;
    double BackRightPower = 0;
    double BackLeftPower = 0;
    double VerticalArmPower = 0;
    double ClawPower = 0;

    /* Define values used in commanding the claw */
    static double clawClose = .5;
    static double clawOpen = -.5;
    static double clawStill = 0;

    /* Define values for teleop bumper control */
    static double nobumper = 1.5;
    static double bumperSlowest = 2;

    /* Define values used in knocking the jewels */
    static double xPosUp = 0;
    static double xPosDown = .5;

    /* Define hardwaremap */
    HardwareMap hwMap = null;

    /* Constructor */
    public MasterHardwareClass() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Hardware
        frontLeftMotor = hwMap.dcMotor.get("FL");
        frontRightMotor = hwMap.dcMotor.get("FR");
        backLeftMotor = hwMap.dcMotor.get("BL");
        backRightMotor = hwMap.dcMotor.get("BR");
        verticalArmMotor = hwMap.dcMotor.get("VAM");
        clawServo =  hwMap.crservo.get("CS");
        gemServo = hwMap.servo.get("gemservo");

        // Set all hardware to default position
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        verticalArmMotor.setPower(0);
        clawServo.setPower(0);


        // Set proper encoder state for all motor
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
