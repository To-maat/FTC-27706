package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo draaiServo = hardwareMap.servo.get("draaiServo");
        Servo wielServo = hardwareMap.servo.get("wielServo");
//        private int joint1Encoder = 0;
//        private DcMotor joint1;


        // Reverse these motors.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize servo positions
        double wielServoPosition = 0.5;
        wielServo.setPosition(wielServoPosition);

        waitForStart();

        if (isStopRequested()) return;
        boolean hangend = false;
        while (opModeIsActive()) {
            // Movement for the mecanum drive
            double y = -gamepad1.left_stick_y; // Y-axis
            double x = gamepad1.left_stick_x * 1.1; // X-axis
            double rx = gamepad1.right_stick_x; // Rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Control arm motor with gamepad2 joystick
            double motorPower;

            if (gamepad2.x) {
                hangend = true;
            }

            if (hangend) {
                motorPower = 1;
                telemetry.addLine("Hangend");
            }
            else{
                motorPower = -gamepad2.left_stick_y;
            }
            armMotor.setPower(motorPower);

//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotor.setTargetPosition(joint1Encoder);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor.setPower(1);
//
//                if (gamepad2.x){
//                    joint1Encoder += 20;
//                }
//                if (gamepad2.y){
//                    joint1Encoder -= 20;
//                }
//                armMotor.setTargetPosition(joint1Encoder);


            // Control draaiServo with gamepad2 joystick
            double joystickX = gamepad2.right_stick_x;
            double draaiServoPosition = (1 - (joystickX + 1) / 2); // Scale -1 to 1 into 0 to 1
            draaiServo.setPosition(draaiServoPosition);

            while (gamepad2.left_bumper) {
                wielServo.setPosition(0);
            }
            while (gamepad2.right_bumper) {
                wielServo.setPosition(1);
            }
            wielServo.setPosition(0.5);

            // Telemetry for debugging
            telemetry.addData("Draai Servo Joystick Input", joystickX);
            telemetry.addData("Draai Servo Position", draaiServoPosition);
            telemetry.addData("Wiel Servo Position", wielServoPosition);
            telemetry.update();
        }
    }
}
