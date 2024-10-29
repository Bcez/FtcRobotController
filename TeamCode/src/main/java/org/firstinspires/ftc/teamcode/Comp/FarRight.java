/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Comp;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Config
@Autonomous(name = "Auto Far Right", group = "Comp")

public class FarRight extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "tce_model_detection.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "tce_model_detection.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "TSE",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public DcMotor motorFrontLeft, motorFrontRight, motorBackRight, motorBackLeft, motor5, motor6;

    public Servo servo1, servo2;

    private final ElapsedTime runtime = new ElapsedTime();

    int i = 0;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.8;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        motorBackLeft = hardwareMap.dcMotor.get("motor1");
        motorBackRight = hardwareMap.dcMotor.get("motor2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        motorFrontRight = hardwareMap.dcMotor.get("motor4");
        //motorArm = hardwareMap.dcMotor.get("motorArm");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        motor5 = hardwareMap.get(DcMotor.class, "actuatorRight");
        motor6 = hardwareMap.get(DcMotor.class, "actuatorLeft");


        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);


        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d");
        motorFrontLeft.getCurrentPosition();
        motorFrontRight.getCurrentPosition();
        motorBackRight.getCurrentPosition();
        motorBackLeft.getCurrentPosition();

        telemetry.update();

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        servo1.setPosition(0);
        sleep(200);
        liftArm();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
//                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.05f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (currentRecognitions.size() > 0 && i < 1) {
                linearDrive(.3, 28.5, 1);
                sleep(500);
                lowerArm();
                dropPixel();
                i++;

                //parking
                linearDrive(.3, -25.5, 1);
                liftArm();
                strafeDrive(.3,99,1);
            }

            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()

    public void startMove() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
        }
        // strafeDrive - assuming arrow is front: right is positive, left is negative
        // linearDrive - assuming arrow is front: front is positive, back is negative
        linearDrive(.3, 30, 1);
    }// end class

    public void strafeDrive(double speed,
                            double target, long timeoutS) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newMotor2Target = motorBackRight.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor1Target = motorBackLeft.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor3Target = motorFrontLeft.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor4Target = motorFrontRight.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            motorBackLeft.setTargetPosition(newMotor1Target);
            motorBackRight.setTargetPosition(-1 * newMotor2Target);
            motorFrontLeft.setTargetPosition(-1 * newMotor3Target);
            motorFrontRight.setTargetPosition(newMotor4Target);

            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed;
            backLeftPower = speed;
            backRightPower = speed;

            motorBackLeft.setPower(frontLeftPower);
            motorBackRight.setPower(frontRightPower);
            motorFrontLeft.setPower(backLeftPower);
            motorFrontRight.setPower(backRightPower);


            while (opModeIsActive() &&
                    (motorBackLeft.isBusy() && motorBackRight.isBusy()) && (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition());
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

            timeoutS = timeoutS * 1000;
            sleep(timeoutS);
        }
    }

    public void linearDrive(double speed,
                            double target, long timeoutS) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        newMotor2Target = motorBackRight.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor1Target = motorBackLeft.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor3Target = motorFrontLeft.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);
        newMotor4Target = motorFrontRight.getCurrentPosition() + (int) (target * COUNTS_PER_INCH / 4);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            motorBackLeft.setTargetPosition(newMotor1Target);
            motorBackRight.setTargetPosition(newMotor2Target);
            motorFrontLeft.setTargetPosition(newMotor3Target);
            motorFrontRight.setTargetPosition(newMotor4Target);

            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed * -1;
            backLeftPower = speed * -1;
            backRightPower = speed;

            motorBackLeft.setPower(frontLeftPower);
            motorBackRight.setPower(frontRightPower);
            motorFrontLeft.setPower(backLeftPower);
            motorFrontRight.setPower(backRightPower);

            while (opModeIsActive() &&
                    (motorBackLeft.isBusy() && motorBackRight.isBusy()) && (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to (STRAFE DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (STRAFE DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d",
                        motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition());
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

            timeoutS = timeoutS * 1000;
            sleep(timeoutS);
        }
    }

    public void liftArm() {
        servo2.setPosition(1);
        motor5.setPower(-0.4);
        motor6.setPower(.4);
        sleep(100);
        motor5.setPower(0);
        motor6.setPower(0);
    }

    public void lowerArm() {
        servo2.setPosition(0);
        motor5.setPower(-0.4);
        motor6.setPower(.4);
        sleep(100);
        motor5.setPower(0);
        motor6.setPower(0);
    }
    public void dropPixel() {
        sleep(1500);
        servo1.setPosition(0.8);
        sleep(500);
    }
}