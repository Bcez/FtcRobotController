//package org.firstinspires.ftc.teamcode.Testing;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Config
//@Autonomous(name="EncoderValueTesting", group="Testing")
////@Disabled
//public class EncoderValueTesting extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//
//    //HardwareBudgetRobot robot = new HardwareBudgetRobot(this);
//    DcMotor arm;
//    CRServo hand;
//
//    @Override
//    public void runOpMode() {
//        //robot.init();
//        arm = hardwareMap.get(DcMotor.class, "Arm");
//        hand = hardwareMap.get(CRServo.class, "Hand");
//        waitForStart();
//
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        int lowHeight = 5800;
//        int maxHeight = 8400;
//
//
//        //Make the arm go up to low height go down to start and then go to middle height
//        // and then back down
//
//        //Go to low height
//        closeHand();
//        sleep(2000);
//        openHand();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Position", arm.getCurrentPosition());
//            telemetry.update();
//
//            //Kill switch
//            if(arm.getCurrentPosition() > 8500) {
//                arm.setPower(0);
//            }
//        }
//
//
//
//    }
//
//
//    public void openHand() {
//        hand.setPower(1);
//        sleep(1750);
//        hand.setPower(0);
//    }
//
//    public void closeHand() {
//        hand.setPower(-1);
//        sleep(1750);
//        hand.setPower(0);
//    }
//
//    public void armMove(double speed,
//                        int target,
//                        double timeoutS) {
//        int newMotor1Target;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            newMotor1Target = arm.getCurrentPosition() + target;
//
//            arm.setTargetPosition(newMotor1Target);
//
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            arm.setPower(Math.abs(speed));
//
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (arm.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to", newMotor1Target);
//                telemetry.addData("Currently at",
//                        arm.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            arm.setPower(0);
//
//
//            // Turn off RUN_TO_POSITION
//            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//    }
//}