package org.firstinspires.ftc.teamcode.Comp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="AngAutonomousBlueClose", group="Comp")
public class AngAutonomousBlueClose extends LinearOpMode {

    public DcMotor motorFrontLeft, motorFrontRight, motorBackRight, motorBackLeft, motorArm;
    public CRServo servoRotate;


    private final ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.8;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5f;
    static final double TURN_SPEED = 0.5;

    @Override


    public void runOpMode() {

        motorBackLeft = hardwareMap.dcMotor.get("motor1");
        motorBackRight = hardwareMap.dcMotor.get("motor2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        motorFrontRight = hardwareMap.dcMotor.get("motor4");
        motorArm = hardwareMap.dcMotor.get("actuatorRight");


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
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0", "Starting at %7d :%7d");
        motorFrontLeft.getCurrentPosition();
        motorFrontRight.getCurrentPosition();
        motorBackRight.getCurrentPosition();
        motorBackLeft.getCurrentPosition();
        motorArm.getCurrentPosition();


        telemetry.update();

        waitForStart();

        strafeDrive(.3, 50, 3);


        // strafeDrive - assuming arrow is front: right is negative, left is positive
        // linearDrive - assuming arrow is front: front is negative, back is positive



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void driveByTime(double power, long time) {

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);

        sleep(2000);


    }

    public void armLiftUp(double speed, double target, long timeoutS) {


        int newMotor5Target;
        double armPower;

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newMotor5Target = motorArm.getCurrentPosition() + (int) (target);

        if (opModeIsActive()) {


            motorArm.setTargetPosition(newMotor5Target);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();


            armPower = speed;
            motorArm.setPower((armPower));

        }

        while (opModeIsActive() && motorArm.isBusy() ) {

            // Display it for the driver.
            telemetry.addData("Running to (LINEAR DRIVE)", "Motor 5: ",
                    newMotor5Target);
            telemetry.addData("Currently at (LINEAR DRIVE)", "Motor 5: ",
                    motorArm.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        motorArm.setPower(0);
        timeoutS = timeoutS * 1000;
        sleep(timeoutS);


    }

    public void linearDrive(double speed,
                            double target, double speed2, double target2, long timeoutS) {
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        int newMotor5Target;
        double armPower;

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //new
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newMotor5Target = motorArm.getCurrentPosition() + (int) (target2);


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
            motorArm.setTargetPosition(newMotor5Target);


            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

            armPower = speed2;
            motorArm.setPower((armPower));


            motorBackLeft.setPower(frontLeftPower);
            motorBackRight.setPower(frontRightPower);
            motorFrontLeft.setPower(backLeftPower);
            motorFrontRight.setPower(backRightPower);

            while (opModeIsActive() &&
                    (motorBackLeft.isBusy() && motorBackRight.isBusy()) && (motorFrontLeft.isBusy() && motorFrontRight.isBusy()) && (motorArm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d Motor 5: %7d",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target, newMotor5Target);
                telemetry.addData("Currently at (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d Motor 5: %7d",
                        motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(), motorArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorArm.setPower(0);

            timeoutS = timeoutS * 1000;
            sleep(timeoutS);
        }
    }

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

//        int newMotor5Target;
//        double armPower;

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        newMotor5Target = motorArm.getCurrentPosition() + (int) (target2);


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
            //motorArm.setTargetPosition(newMotor5Target);



            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.  (these values are for strafing right)
            runtime.reset();


            frontLeftPower = speed;
            frontRightPower = speed;
            backLeftPower = speed;
            backRightPower = speed;

            //new
            // armPower = speed2;
            //motorArm.setPower((armPower));

            motorBackLeft.setPower(frontLeftPower);
            motorBackRight.setPower(frontRightPower);
            motorFrontLeft.setPower(backLeftPower);
            motorFrontRight.setPower(backRightPower);


            while (opModeIsActive() &&
                    (motorBackLeft.isBusy() && motorBackRight.isBusy()) && (motorFrontLeft.isBusy() && motorFrontRight.isBusy()) ) {

                // Display it for the driver.
                telemetry.addData("Running to (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d ",
                        newMotor1Target, newMotor2Target, newMotor3Target, newMotor4Target);
                telemetry.addData("Currently at (LINEAR DRIVE)", "Motor 1: %7d Motor 2: %7d Motor 3: %7d Motor 4: %7d ",
                        motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            //motorArm.setPower(0);


            timeoutS = timeoutS * 1000;
            sleep(timeoutS);
        }
    }
}



































//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = motorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = motorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            motorBackLeft.setTargetPosition(newLeftTarget);
//            motorFrontLeft.setTargetPosition(newLeftTarget);
//            motorBackRight.setTargetPosition(newRightTarget);
//            motorFrontRight.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            motorFrontLeft.setPower(Math.abs(speed));
//            motorFrontRight.setPower(Math.abs(speed));
//            motorBackLeft.setPower(Math.abs(speed));
//            motorBackRight.setPower(Math.abs(speed));
//
//
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (motorBackLeft.isBusy() && motorFrontLeft.isBusy()&& motorFrontRight.isBusy()&&motorBackRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d");
//                motorFrontRight.getCurrentPosition();
//                motorFrontLeft.getCurrentPosition();
//                motorBackLeft.getCurrentPosition();
//                motorBackRight.getCurrentPosition();
//
//                telemetry.update();
//            }
//
//
//
//            // Stop all motion;
//            motorBackLeft.setPower(0);
//            motorFrontLeft.setPower(0);
//            motorBackRight.setPower(0);
//            motorFrontRight.setPower(0);
//
//
//
//            // Turn off RUN_TO_POSITION
//            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//
//
//    }

//    public void strafeDrive(double speed,
//                              double frontInches, double backInches,
//                              double timeoutS) {
//        int newFrontLeftTarget;
//        int newFrontRightTarget;
//        int newBackLeftTarget;
//        int newBackRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            newBackLeftTarget = motorBackLeft.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
//            newBackRightTarget = motorBackRight.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
//            newFrontLeftTarget = motorFrontLeft.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
//            newFrontRightTarget = motorBackRight.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
//            motorBackLeft.setTargetPosition(newBackLeftTarget);
//            motorBackRight .setTargetPosition(newBackRightTarget);
//            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
//            motorFrontRight.setTargetPosition(newFrontRightTarget);
//
//
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            motorBackRight.setPower(Math.abs(speed));
//            motorBackLeft.setPower(Math.abs(speed));
//            motorFrontLeft.setPower(Math.abs(speed));
//            motorFrontRight.setPower(Math.abs(speed));
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (motorBackLeft.isBusy() && motorBackRight.isBusy()) && (motorFrontLeft.isBusy() && motorFrontRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newFrontLeftTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                        motorBackLeft.getCurrentPosition(), motorFrontLeft.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            motorFrontRight.setPower(0);
//            motorFrontLeft.setPower(0);
//            motorBackRight.setPower(0);
//            motorBackLeft.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//    }
//}
