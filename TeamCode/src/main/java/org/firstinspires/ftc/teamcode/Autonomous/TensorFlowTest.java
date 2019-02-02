/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Teleops.HardwareMap;

import java.util.List;


@Autonomous(name = "TensorFlow Crater", group = "Pushbot")
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "AbWq5Uv/////AAAAGcM6elMmVUOogS1JXLiPuvxUPJgstfq86yvjnKb87ZG0oftSUkO7FUXo+LPZdGe3ytBqkwQmXV6b0hiAMotK9TAX//BaE8tYQe0cJQzMPk5jjMAmLbZgZ1p3V9EQzp59pYvYvBMYzoNw7YzlpMNC3GzmXd40NyecOmx8Q6lp/tQikXO0yKGQLIoJpKtGfoVkxpmyCx/u4/6FYBAGyvZt8I8mz3UtGl/Yf366XKgNXq26uglpVfeurmB/cV5RzMVdVDTdyE/2yLqjalrAKgL2CZFv3iY/MnxW+pIyJUHbXUQVCUoB8SqULq7u948Vx+5w5ObesVFNzZ3jbBTBHwUWbpaJAZFGjmD1dRaVdS/GK74x";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    public boolean goldFound = false;
    public boolean path1 = false, path2 = false, path3 = false;


    HardwareMap robot = new HardwareMap();
    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Encoders Reset");    //
        telemetry.update();

//        sleep(500);

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        robot.armMotor.setTargetPosition(5750);

        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();

        // Turn On RUN_TO_POSITION

        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);

        sleep(2500);     // pause for motor to move

        telemetry.update();

        TurnRight(18);

        GO(3, 0.5);

        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && !path1 && !path2 && !path3) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                     //   telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 1) {

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    sleep(250);
                                    telemetry.addData("gold", "1st");
                                    GO(27.5, .7);
                                    path1 = true;
                                    goldFound = true;
                                } else {
                                    telemetry.addData("silver", "1st");
                                }
                            }

                            if(!goldFound) {
                                sleep(100);
                                TurnLeft(19);
                                sleep(1500);

                                updatedRecognitions.clear();
                                updatedRecognitions = tfod.getUpdatedRecognitions();

                                for (Recognition recognition : updatedRecognitions) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        sleep(250);
                                        telemetry.addData("gold", "2nd");
                                        path2 = true;
                                        GO(29, .75);
                                        goldFound = true;
                                    }
                                }
                            }
                            if(!goldFound) {
                                telemetry.addData("silver", "2nd");
                                TurnLeft(18);
                                GO(35, .75);
                                path3 = true; }


                            telemetry.update();
                        }
                    }
                }
            }

        }

        if (tfod != null) {
            tfod.shutdown(); }

        if(path1)
        {
            telemetry.addData("path", "1st position");
            telemetry.update();
            TurnRight(37.5); //turn w/ mineral
            GO(37, 1); //depot set-up
            TurnRight(15); // depot turn
            GO(38.5, 1);
            robot.mServo.setPosition(.8);
            sleep(500);
            GoBack(70, 1);
            TurnRight(7);
            GoBack(15, 0.75);
        }
        else if(path2)
        {
            GoBack(11.5, .75);
            telemetry.addData("path", "2nd position");
            telemetry.update();
            TurnRight(41.5);
            GO(61, 1);
            TurnRight(20);
            GO(30, 1);
            robot.mServo.setPosition(.8);
            sleep(250);
            GoBack(70, 1);
            TurnRight(7);
            GoBack(15, 0.75);
        }
        else if(path3)
        {
            GoBack(10,.75);
            telemetry.addData("path", "3rd position");
            telemetry.update();
            TurnRight(53.5);
            GO(69, 1);
            TurnRight(21);
            GO(33, 1);
            robot.mServo.setPosition(.8);
            sleep(250);
            GoBack(72, 0.8);
        }

        sleep(10000);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
        sleep(500);

        robot.rightDrive.setPower(-.5);
        robot.leftDrive.setPower(.5);

        while (robot.leftDrive.getCurrentPosition() > -degrees*24.444 && opModeIsActive()) {
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


    public void TurnLeft(double degrees) {
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Working", "Left: %7d Right: %7d Arm: %7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.armMotor.getCurrentPosition());
        telemetry.update();
        sleep(500);

        robot.rightDrive.setPower(.5);
        robot.leftDrive.setPower(-.5);

        while (robot.leftDrive.getCurrentPosition() < degrees*24.444 && opModeIsActive()) {
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

    public void GO(double inches, double power)
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
        sleep(500);

        robot.rightDrive.setPower(power);
        robot.leftDrive.setPower(power);

        while (robot.leftDrive.getCurrentPosition() > -inches*47.619 && opModeIsActive()) {
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
    public void GoBack(double inches, double power)
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
        sleep(500);

        robot.rightDrive.setPower(-power);
        robot.leftDrive.setPower(-power);

        while (robot.leftDrive.getCurrentPosition() < inches*47.619 && opModeIsActive()) {
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

