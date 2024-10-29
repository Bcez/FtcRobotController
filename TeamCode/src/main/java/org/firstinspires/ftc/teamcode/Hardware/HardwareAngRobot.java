package org.firstinspires.ftc.teamcode.Hardware;


import static android.os.SystemClock.sleep;

import android.graphics.Camera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *   Budget: Mecanum/Holometric Drive
 */

public class HardwareAngRobot {

    /* Constants */

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Public Opmode Members */
    public DcMotor
            motor1,motor2, motor3, motor4, motor5, motor6, motor7;

    public Servo servo1, servo2;
    BNO055IMU imu;


    public HardwareAngRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        /* Drive Hardware */
          motor1 = myOpMode.hardwareMap.get(DcMotor.class, "motor1");
          motor2 = myOpMode.hardwareMap.get(DcMotor.class, "motor2");
          motor3 = myOpMode.hardwareMap.get(DcMotor.class, "motor3");
          motor4 = myOpMode.hardwareMap.get(DcMotor.class, "motor4");
          motor5 = myOpMode.hardwareMap.get(DcMotor.class, "actuatorRight");
          motor6 = myOpMode.hardwareMap.get(DcMotor.class, "actuatorLeft");
          motor7 = myOpMode.hardwareMap.get(DcMotor.class, "actuatorFwd");
//          motor8 = myOpMode.hardwareMap.get(DcMotor.class, "actuatorWrist");
//          servo1 = myOpMode.hardwareMap.get(Servo.class, "servo1");
//          motorArm = myOpMode.hardwareMap.get(DcMotor.class, "arm");



        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setPower(0); //
        motor2.setPower(0); //
        motor3.setPower(0); //
        motor4.setPower(0); //
        motor5.setPower(0); //
        motor6.setPower(0); //
        motor7.setPower(0); //
        /* Intake Hardware */

        /* Servos */
        servo1 = myOpMode.hardwareMap.get(Servo.class, "servo1");
        servo2 = myOpMode.hardwareMap.get(Servo.class, "servo2");

        //servo2.setPosition(1);
        motor5.setPower(-0.4);
        motor6.setPower(.4);
        sleep(100);
        motor5.setPower(0); //
        motor6.setPower(0); //
    }



    /* Intake Methods */

}
