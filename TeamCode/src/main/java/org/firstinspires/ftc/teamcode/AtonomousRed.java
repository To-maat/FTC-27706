package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Red", group = "Autonoom")
public class AtonomousRed extends LinearOpMode {

    // Define motoren
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;

    @Override
    public void runOpMode() {
        // Declare motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        Servo draaiServo = hardwareMap.servo.get("draaiServo");
        Servo wielServo = hardwareMap.servo.get("wielServo");

        // Reverse these motors.
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            driveForward(100);

            sleep(500);

            driveRight(500);

            sleep(500);

            driveForward(1600);

            driveRight(300);

            sleep(500);

            driveBackwards(1600);

            sleep(500);

            driveForward(1700);

            driveRight(500);

            sleep(500);

            driveBackwards(1600);

            sleep(500);

            driveForward(300);

            sleep(500);

            Arm(1000);

            Servo(draaiServo, 0.5, 1000);

            Arm(6000);
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
    private void driveRight(long tijd) {
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);
        sleep(tijd);
        stopMotoren();
    }

    private void Arm(long tijd) {
        armMotor.setPower(-0.5);
        sleep(tijd);
        stopMotoren();
    }

    private void Servo(Servo servo, double positie, long wachttijd) {
        servo.setPosition(positie); // Stel de servo in op de opgegeven positie
        sleep(wachttijd); // Wacht zodat de servo tijd heeft om te bewegen
    }

    // Function to stop the motors
    private void stopMotoren() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);
    }
}
