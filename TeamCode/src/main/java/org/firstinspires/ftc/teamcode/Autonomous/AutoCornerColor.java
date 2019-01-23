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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleops.HardwareMap;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//NEW autonomous facing the depot (w/ color path)
@Autonomous(name = "TEST Depot Autonomous", group = "Pushbot")
//@Disabled
public class AutoCornerColor extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap robot = new HardwareMap();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    int max = 23;
    int min = 15;

    /*static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5; */

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Encoders Reset");    //
        telemetry.update();

        sleep(500);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset

        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //  encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        // robot.rightClaw.setPosition(0.0);
        robot.armMotor.setTargetPosition(5750);

        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();

        // Turn On RUN_TO_POSITION
        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);

        sleep(3000);     // pause for servos to move

        // Wait for the game to start (driver presses PLAY)


        telemetry.update();


/*
        robot.leftDrive.setPower(.5);
        sleep(500);
        robot.leftDrive.setPower(0);
*/
/*
        robot.rightDrive.setTargetPosition(-20000);
        robot.leftDrive.setTargetPosition(20000);
        sleep(500);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        robot.rightDrive.setPower(-1);
        robot.leftDrive.setPower(-1);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.leftDrive.setPower(0);
        //robot.rightDrive.setPower(0);
        while (robot.leftDrive.isBusy() || robot.rightDrive.isBusy()) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
               robot.rightDrive.getCurrentPosition(),
               robot.armMotor.getCurrentPosition());
              telemetry.update();
             }

        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
*/
        TurnRight(35);

        telemetry.addData("waiting", "waiting");
        telemetry.update();
        sleep(100);


        GO(24);

        TurnRight(24);

        robot.csServo.setPosition(.25);

        //Math.max(robot.color1.blue(), robot.color2.blue());

        GoBackSlow(11);

//      max = 25, min = 15
        if (Math.max(robot.color1.blue(), robot.color2.blue()) < max && Math.max(robot.color1.blue(), robot.color2.blue()) > min)
        {
            robot.csServo.setPosition(.1);
            sleep(250);
            robot.csServo.setPosition(.8);
            GO(20);
        }
        else
        {
            GoBackSlow(19);
            sleep(250);

            if (Math.max(robot.color1.blue(), robot.color2.blue()) < max && Math.max(robot.color1.blue(), robot.color2.blue()) > min)
            {
                robot.csServo.setPosition(.1);
                sleep(250);
                robot.csServo.setPosition(.8);
                GO(42);
            }
            else
            {
                GoBackSlow(21);
                sleep(250);
                robot.csServo.setPosition(.1);
                sleep(500);
                robot.csServo.setPosition(.8);
                GO(55);
            }
        }


/*      sleep(250);
        GoBackSlow(19);
        sleep(250);
        GoBackSlow(19);
        sleep(250);
*/

        robot.csServo.setPosition(.7);
        telemetry.addData("Servo", "Servo: %7f", robot.csServo.getPosition());
        telemetry.update();
//      robot.csServo.setPosition(0.25);
        sleep(500);
        telemetry.addData("Servo", "Servo: %7f", robot.csServo.getPosition());
        telemetry.update();


      //  TurnLeft(25);

//        GO(70);
//-------------------------------------------------------------
        TurnRight(35);

        GoBack(60);

        //TurnRight(-5);

        TurnRight(75);

        robot.mServo.setPosition(0.2);

        GoBack(80);
//--------------------------------------------------

//-----        GoBack(85);

//        TurnRight(45);

//        GoBack(36);

//        GO(74);

        sleep(15000);
    }

    public void GO(double inches)
    {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(1);
        robot.leftDrive.setPower(1);

        while (robot.leftDrive.getCurrentPosition() > -inches*47.619) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void GoBackSlow(double inches)
    {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);               // Change to USING if does not work
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(-.3);
        robot.leftDrive.setPower(-.3);
        while (robot.rightDrive.getCurrentPosition() < inches*47.619 && robot.leftDrive.getCurrentPosition() < inches*47.619) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void GoBack(double inches)
    {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(-1);
        robot.leftDrive.setPower(-1);

        while (robot.leftDrive.getCurrentPosition() < inches*47.619) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        //robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void TurnRight(double degrees)
    {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(-.1);
        robot.leftDrive.setPower(.1);

        while (robot.leftDrive.getCurrentPosition() > -degrees*24.444) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void TurnLeft(int degrees) {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(.1);
        robot.leftDrive.setPower(-.1);

        while (robot.leftDrive.getCurrentPosition() > degrees*24.444) {
            telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

}

