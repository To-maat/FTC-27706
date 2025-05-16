package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    //Position change of arm
    private final int POSITION_CHANGE = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor"); // Use the correct name from your hardware configuration
        Servo draaiServo = hardwareMap.servo.get("draaiServo");
        Servo wielServo = hardwareMap.servo.get("wielServo");

        // Reverse these motors.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Arm position constants (adjust to your needs)
        int armUpPosition = 3000;
        int armDownPosition = 0;

        //Initialize the arm
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        boolean isWielServoActive = false;
        double wielServoPosition = 0.5; // 0.5 betekent stilstand voor een continuous rotation servo

        waitForStart();

        if (isStopRequested()) return;

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

            // If the A button is pressed, raise the arm
            if (gamepad2.a) { //Use gamepad 2 here for the arm
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad2.b) {  //Use gamepad 2 here for the arm
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            if (gamepad2.y) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int currentPosition = 0;
                armMotor.setTargetPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            if (gamepad2.x) {
                armMotor.setTargetPosition(7000);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // UP ARROW: Incremental position change
            if (gamepad2.dpad_up) { //Use gamepad 2 here for the arm
                int currentPosition = armMotor.getCurrentPosition();
                armMotor.setTargetPosition(currentPosition + POSITION_CHANGE);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }
            //new comment
            // DOWN ARROW: Incremental position change
            if (gamepad2.dpad_down) { //Use gamepad 2 here for the arm
                int currentPosition = armMotor.getCurrentPosition();
                armMotor.setTargetPosition(currentPosition - POSITION_CHANGE);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }


            // Control draaiServo with gamepad2 right stick
            double joystickX = gamepad2.right_stick_x;
            if (Math.abs(joystickX) < 0.05) { // Waarden tussen -0.05 en 0.05 worden genegeerd
                joystickX = 0;
            }
            double draaiServoPosition = (1 - (joystickX + 1) / 2.0); // Scale -1 to 1 into 0 to 1
            draaiServo.setPosition(draaiServoPosition);

            if (gamepad2.left_bumper) {
                wielServoPosition = 0;  // Draai linksom
                isWielServoActive = true;
            } else if (gamepad2.right_bumper) {
                wielServoPosition = 1;  // Draai rechtsom
                isWielServoActive = true;
            } else if (isWielServoActive) {
                wielServoPosition = 0.5;  // Stop servo als er geen knop wordt ingedrukt
                isWielServoActive = false;
            }


// Zet de servo alleen als de positie veranderd is
            wielServo.setPosition(wielServoPosition);

            // Telemetry for debugging
            telemetry.addData("Draai Servo Joystick Input", joystickX);
            telemetry.addData("Draai Servo Position", draaiServoPosition);
            telemetry.addData("Wiel Servo Position", wielServoPosition);
            telemetry.addData("Arm Encoder Position", armMotor.getCurrentPosition());
            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            telemetry.update();
        }
    }
}