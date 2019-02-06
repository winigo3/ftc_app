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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;     //UNCOMMENT IF MESS UP HAPPENS
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//Tele-op file
@TeleOp(name="Telop Tank", group="BACON")
//@Disabled
public class TeleopTank_Drive extends LinearOpMode {



    double globalAngle = 0;
    Orientation lastAngles = new Orientation();
    Orientation angles;
    /* Declare OpMode members. */
    HardwareMap robot = new HardwareMap();              // Use a K9'shardware
    //double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    //double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    //final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    //final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        //robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.craterArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.leftDrive.setPower(left/3);    //Add /2 for Discovery Museum
            robot.rightDrive.setPower(right/3);

            if (gamepad1.x)
            {
                robot.leftDrive.setDirection(DcMotor.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotor.Direction.REVERSE);
            }
            if (gamepad1.b)
            {
                robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
            }
            //   Use gamepad bumpers to raise and lower the arm
            if (gamepad1.left_bumper)
                robot.armMotor.setPower(-.5);
            else if (gamepad1.right_bumper)
                robot.armMotor.setPower(.5);
            else
                robot.armMotor.setPower(0);


            if (gamepad2.dpad_up)
                robot.aServo.setPosition(.2);
            else if (gamepad2.dpad_down)
                robot.aServo.setPosition(.9);
            else if(gamepad2.dpad_left)
                robot.aServo.setPosition(.5);

            if(gamepad2.right_trigger > 0)
                robot.craterArm.setPower(gamepad2.right_trigger*0.7);
            else if (gamepad2.left_trigger > 0)
                robot.craterArm.setPower(-gamepad2.left_trigger*0.7);
            else
                robot.craterArm.setPower(0);

               // robot.rightDrive.setPower(0);

           /* if (gamepad1.right_bumper)
                robot.armMotor.setPower(.5);
            else
                    robot.armMotor.setPower(0);
            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;

            // Move both servos to new position.
           armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
                    robot.arm.setPosition(armPosition);
                    clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
                    robot.claw.setPosition(clawPosition);

                    // Send telemetry message to signify robot running;

                    telemetry.addData("arm",   "%.2f", armPosition);
                    telemetry.addData("claw",  "%.2f", clawPosition);
                    */
            //telemetry.addData("left", "%.2f", left);
            //telemetry.addData("right", "%.2f", right);
 /*           telemetry.addData("Working",  "Left: %7d Right: %7d Arm: %7d",
                    robot.leftDrive.getCurrentPosition(),
                    robot.rightDrive.getCurrentPosition(),
                    robot.armMotor.getCurrentPosition());
            telemetry.update();


            //Pause for 40 mS each cycle = update 25 times a second.
*/
       //     angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
/*
            telemetry.addData("imu angle: ", angles.firstAngle);
            telemetry.update();
*/
            sleep(40);
        }
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);


        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void TurnRight(double degrees)
    {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(-.35);
        robot.leftDrive.setPower(.35);

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

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(1000);

        robot.rightDrive.setPower(.35);
        robot.leftDrive.setPower(-.35);

        while (robot.leftDrive.getCurrentPosition() < degrees*24.444) {
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

}
