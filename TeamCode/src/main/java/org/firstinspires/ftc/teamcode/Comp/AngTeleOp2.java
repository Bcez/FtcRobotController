/*
 * MECANUM/HOLOMETRIC MODE TELE OP MODE
 */

package org.firstinspires.ftc.teamcode.Comp;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Hardware.HardwareAngRobot;

import java.util.ArrayList;
// test also hello play hollow knight
@Config
@TeleOp(name="AngTeleOp2", group="Comp")
public class AngTeleOp2 extends LinearOpMode {

    HardwareAngRobot robot = new HardwareAngRobot(this);

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialization Complete");

        boolean clawOpen = false;
        int i = 0;

        telemetry.addData("i", i);

        robot.init();

        telemetry.update();
        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        while (opModeIsActive()) {
            if (i < 1 || gamepad2.b) { // starts at beginning position
                robot.motor5.setPower(-0.3);
                robot.motor6.setPower(.3);
                sleep(100);
                robot.motor5.setPower(0); //
                robot.motor6.setPower(0); //
                i++;
            }

            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x * -1;

            if (gamepad1.dpad_up) {
                x = 1;
            }

            if (gamepad1.dpad_down) {
                x = -1;
            }

            if (gamepad1.dpad_left) {
                y = -1;
            }
            if (gamepad1.dpad_right) {
                y = 1;
            }

            //Used to ensure same ratio and contain values between [-1,1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double pitchArm = 0;
            double spinnyTime = 0;
            double actuatorExt = 0;

            double throttle_control = 0.6;
            double slowDown1 = 1; // i kind of like .7 default speed more but whatever ://///////////////
            double slowDown2 = 1;
            double slowDown3 = 1;

            if ((Math.abs(gamepad2.left_stick_y) + Math.abs(gamepad2.right_stick_y)) > 0) {
                pitchArm += gamepad2.left_stick_y;
                spinnyTime -= gamepad2.right_stick_y;
                pitchArm *= 0.6;
            }

            if (gamepad2.left_trigger + gamepad2.right_trigger > 0) { // negative values extend pos values retract
                actuatorExt += gamepad2.left_trigger - gamepad2.right_trigger;

                robot.motor5.setPower(-0.3 * Math.abs(actuatorExt));
                robot.motor6.setPower(0.3 * Math.abs(actuatorExt));
            }

//            if (gamepad2.left_bumper && !clawOpen) {
//                robot.servo1.setPosition(0.8);
//                clawOpen = true;
//            }
//
//            if (gamepad2.right_bumper && clawOpen) {
//                robot.servo1.setPosition(0);
//                clawOpen = false;
           // }

//            if(gamepad2.y) {
//                // move to 0 degrees.
//                robot.servo1.setPosition(-1.0);
//            }
//            if (gamepad2.x) {
//                // move to 90 degrees.
//                robot.servo1.setPosition(1.0);
//            }

            if (gamepad1.right_trigger > 0)
                slowDown1 -= 0.5;

            if (gamepad2.a)
                slowDown2 -= 0.5;


            if (gamepad2.right_bumper)
                slowDown3 -= 0.2;

            robot.motor1.setPower(frontLeftPower * throttle_control * slowDown1);
            robot.motor2.setPower(backLeftPower * throttle_control * slowDown1);
            robot.motor3.setPower(frontRightPower * throttle_control * slowDown1);
            robot.motor4.setPower(backRightPower * throttle_control * slowDown1);

            robot.motor5.setPower(pitchArm * throttle_control * slowDown3);
            robot.motor6.setPower(-1 * pitchArm * throttle_control* -slowDown3);
            robot.motor7.setPower(actuatorExt * slowDown2);


//            robot.motor5.setTargetPosition((int) (robot.motor5.getCurrentPosition() * pitchArm * throttle_control));
//            robot.motor6.setTargetPosition((int) ((robot.motor6.getCurrentPosition() * pitchArm * -1 * throttle_control)));

            //up mode
            if (Math.abs(gamepad2.right_stick_y) > 0)
                robot.servo2.setPosition(spinnyTime * 1.5);

            //down mode
            if (Math.abs(gamepad2.right_stick_y) < 0)
                robot.servo2.setPosition(spinnyTime * -1.5);


            double servoPos;

            servoPos = robot.servo2.getPosition();

//            robot.motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("actuatorExt", actuatorExt);
            telemetry.addData("spinny values:", servoPos + " " + spinnyTime + " " + gamepad2.right_stick_y);
            telemetry.addData("armMotors", "left: " + robot.motor5.getCurrentPosition() + ", right: " + ( -1 * robot.motor6.getCurrentPosition()));
            telemetry.addData("i", i);

            telemetry.update();
        }
    }
}