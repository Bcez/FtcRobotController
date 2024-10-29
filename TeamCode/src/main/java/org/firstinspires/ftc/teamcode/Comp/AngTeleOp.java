///*
// * MECANUM/HOLOMETRIC MODE TELE OP MODE
// */
//
//package org.firstinspires.ftc.teamcode.Comp;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import org.firstinspires.ftc.teamcode.Hardware.HardwareAngRobot;
//
//import java.util.ArrayList;
//
//@Config
//@TeleOp(name="AngTeleOp", group="Comp")
//public class AngTeleOp extends LinearOpMode {
//
//    HardwareAngRobot robot = new HardwareAngRobot(this);
//
//    @Override
//    public void runOpMode() {
//        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("Status", "Initialization Complete");
//
//
//        robot.init();
//
//        telemetry.update();
//        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        waitForStart();
//        while (opModeIsActive()) {
//
//            double x = -gamepad1.left_stick_y;
//            double y = gamepad1.left_stick_x * 1.1;
//            double rx = -gamepad1.right_stick_x * -1;
//
//            if(gamepad1.dpad_up) {
//                x = 1;
//            }
//
//            if(gamepad1.dpad_down) {
//                x = -1;
//            }
//
//            if(gamepad1.dpad_left) {
//                y = -1;
//            }
//            if(gamepad1.dpad_right) {
//                y = 1;
//            }
//
//            //Used to ensure same ratio and contain values between [-1,1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//
//
//            double throtte_control = 0.6;
//            double slowDown = 1;
//            if(gamepad1.right_trigger > 0)
//                slowDown -= 0.5;
//
//            robot.motor1.setPower(frontLeftPower*throtte_control*slowDown);
//            robot.motor2.setPower(backLeftPower*throtte_control*slowDown);
//            robot.motor3.setPower(frontRightPower*throtte_control*slowDown);
//            robot.motor4.setPower(backRightPower*throtte_control*slowDown);
//            //robot.hand.setPosition(gripPower);
///*
//            telemetry.addData("frontLeft:", frontLeftPower);
//            telemetry.addData("backLeft:", backLeftPower);
//            telemetry.addData("frontRight:", frontRightPower);
//            telemetry.addData("backRight:", backRightPower);
//            telemetry.update();
//*/
//
////            if (gamepad2.left_stick_y * -1 > 0) { //up
////                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() - 100);
////                robot.motorArm.setPower(.75);
////                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            }
////            else if (gamepad2.left_stick_y * -1 < 0) { //down
////                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() + 100);
////                robot.motorArm.setPower(.5);
////                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            }
////
////            if(gamepad2.a) {
////                robot.motorArm.setTargetPosition(-2750);
////                robot.motorArm.setPower(.5); //TODO: change?
////                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            }
//            /* INTAKE */
///*            else if (robot.motorArm.getCurrentPosition() < -2175) {
//                robot.motorArm.setTargetPosition(-2150);
//                robot.motorArm.setPower(.5); //TODO: change?
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else if (robot.motorArm.getCurrentPosition() > 25) {
//                robot.motorArm.setTargetPosition(0);
//                robot.motorArm.setPower(.5); //TODO: change?
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//*/
//        //    if(gamepad2.right_bumper) { //close
//       //         robot.servo1.setPosition(0.5);
//            }
//      //      else if(gamepad2.left_bumper) {  //open
//         //       robot.servo1.setPosition(-0.5);
//            }
//
//       //     boolean right_trigger = gamepad2.right_trigger > 0.0;
//       //     boolean left_trigger = gamepad2.left_trigger > 0.0;
//
//            //if (right_trigger) {
//       //         robot.servo2.setPosition(0.9);
//            }
//            //if (left_trigger) {
//       //         robot.servo2.setPosition(-0.5);
//            //}
//            //if (gamepad2.b) {
//      //          robot.servo2.setPosition(0.5);
//           // }
//            //if (gamepad2.y) {
//         //       robot.servo2.setPosition(0.0);
//           // }
//
//           /* double wristChange = -gamepad2.right_stick_y * 0.02;
//
//            if (Math.abs(gamepad2.right_stick_y) > 0) {
//                if (robot.servo2.getPosition() == (robot.servo2.getPosition() + wristChange)) { // servo goes indefinitely for now, for comp purposes switch to power servo -josh
//                    break;
//                }
//
//                robot.servo2.setPosition(robot.servo2.getPosition() + wristChange);
//                telemetry.addData("joystick_y", wristChange);
//            }
//
//            */
//            //if (gamepad2.x)
//     //           robot.servo2.setPosition(0.4772222); telemetry.addData("Servo2", "capture position");
//
//
//            //if (gamepad2.right_stick_button)
//        //        robot.servo2.setPosition(0);
//
//
//            //motorTelemetry();
//           // wristTelemetry();
//        //}
//  //  }
//
//    /*public void wristTelemetry() {
//        telemetry.addData("Servo 2", robot.servo2.getPosition());
//        telemetry.addData("move", (robot.servo2.getPosition() + (-gamepad2.right_stick_y * 0.01)));
//    }*/
//   // public void motorTelemetry() {
//   //     telemetry.addData("Arm", robot.motorArm.getCurrentPosition());
//   //     telemetry.update();
//  //  }
////}
//
///*
//            // double throtte_control = 0.5;
//            double slowDown1 = 1;
//            double slowDown2 = 0.5;
//            if(gamepad1.right_trigger > 0 ) {
//                slowDown1 -= 0.50;
//            }
//
//            if(gamepad2.right_trigger > 0 ) {
//                slowDown2 -= 0.25;
//            }
//
//            robot.motor1.setPower(frontLeftPower*throtte_control*slowDown1*-1);
//            robot.motor2.setPower(backLeftPower*throtte_control*slowDown1*-1);
//            robot.motor3.setPower(frontRightPower*throtte_control*slowDown1*-1);
//            robot.motor4.setPower(backRightPower*throtte_control*slowDown1*-1);
//
//
////            if (right_stick_y < 0) {
////                robot.servo2.setPower(.8);
////                telemetry.addData("Status", "Rotating Servo Clockwise");
////            }
////
////            if (right_stick_y > 0) {
////                robot.servo2.setPower(-0.8);
////                telemetry.addData("Status", "Rotating Servo Clockwise");
////            }
////
////            if (right_stick_y > 0) {
////                robot.servo2.setPower(0.1);
////            }
////
////            if (right_stick_y < 0) {
////                robot.servo2.setPower(-0.1);
////            }
////
////            if (gamepad2.right_bumper) {
////                robot.servo1.setPower(.6);
////                telemetry.addData("Status", "Rotating Servo Clockwise");
////            }
////
////            if (gamepad2.left_bumper) {
////                robot.servo1.setPower(-.6);
////                telemetry.addData("Status", "Rotating Servo Clockwise");
////            }
////
////            robot.servo1.setPower(0);
////            telemetry.addData("Status", "Stopping Servo");
////
////            //Plane Launcher
////
////            if (gamepad2.x) {
////                robot.servo3.setPosition(0.0);
////                telemetry.addData("Status", "Launching Plane");
////            } else if (Math.abs(gamepad2.left_trigger) > 0) {
////                robot.servo3.setPosition(2.0);
////            }
//
//
//            //Arm code - fast
//
//
//            if (gamepad2.a) {
//                robot.motorArm.setTargetPosition(0);
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            //fast
//            if (gamepad2.left_stick_y < 0) {
//                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() + 150);
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            if (gamepad2.left_stick_y > 0) {
//                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() - 150);
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            //slow
//            if (gamepad2.dpad_up) {
//                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() + 25);
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            if (gamepad2.dpad_down) {
//                robot.motorArm.setTargetPosition(robot.motorArm.getCurrentPosition() - 25);
//                robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//
//        }
//    }
//}   */
