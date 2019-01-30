/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareMap
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor armMotor = null;
    public DcMotor craterArm = null;
    public Servo    csServo       = null;
    public ColorSensor color1 = null;
    public ColorSensor color2 = null;
    public Servo mServo = null;
    public Servo aServo = null;
    public Servo dumperArm = null;
    public BNO055IMU imu;

    //public Servo cs2Servo = null;

    //public Servo    claw        = null;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    com.qualcomm.robotcore.hardware.HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(com.qualcomm.robotcore.hardware.HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "l");
        rightDrive = hwMap.get(DcMotor.class, "r");
        armMotor = hwMap.get(DcMotor.class, "arm");
        craterArm = hwMap.get(DcMotor.class, "ca");
        dumperArm = hwMap.servo.get("dwn");
        dumperArm.setPosition(.8);
//        csServo = hwMap.servo.get("cs");
//        csServo.setPosition(.9);
        mServo = hwMap.servo.get("ms");
        mServo.setPosition(0);
        aServo = hwMap.servo.get("a");
        aServo.setPosition(.4);
//        color2 = hwMap.colorSensor.get("c2");
//        color1 = hwMap.colorSensor.get("c1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        craterArm.setPower(0);
        //armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(3355);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craterArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        craterArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define and initialize ALL installed servos.
       // arm  = hwMap.get(Servo.class, "arm");
       // claw = hwMap.get(Servo.class, "claw");
       // arm.setPosition(ARM_HOME);
       // claw.setPosition(CLAW_HOME);
        //test
    }
}
