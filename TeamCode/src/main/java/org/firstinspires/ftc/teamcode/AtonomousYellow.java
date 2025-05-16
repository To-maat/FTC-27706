package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Yellow", group = "Autonoom")
public class AtonomousYellow extends LinearOpMode {
    //new comment
    // Define motoren
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        // Declare motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");
        Servo draaiServo = hardwareMap.servo.get("draaiServo");
        Servo wielServo = hardwareMap.servo.get("wielServo");

        // Reverse these motors.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            driveForward(100);

            sleep(500);

            driveLeft(500);

            sleep(500);

            driveForward(1600);

            driveLeft(300);

            sleep(500);

            driveBackwards(1600);

            sleep(500);

            driveForward(1700);

            driveLeft(500);

            sleep(500);

            driveBackwards(1600);

            sleep(500);

            driveRight(3000 );
        }
    }

    // Function to drive forward
    private void driveForward(long tijd) {
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        sleep(tijd);
        stopMotoren();
    }

    // Function to drive backwards
    private void driveBackwards(long tijd) {
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);
        sleep(tijd);
        stopMotoren();
    }

    // Function to drive to the right
    private void driveLeft(long tijd) {
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);
        sleep(tijd);
        stopMotoren();
    }

    private void driveRight(long tijd) {
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);
        sleep(tijd);
        stopMotoren();
    }
    // Function to stop the motors
    private void stopMotoren() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
