package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FTCLibTest extends LinearOpMode {
    private PIDController controller;

    // PID Coefficients
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Position constants from MecanumTeleOp
    private static final int ARM_UP_POSITION = 3000;
    private static final int ARM_DOWN_POSITION = 0;
    private static final int POSITION_CHANGE = 200;

    // Tolerance for position control
    private static final double POSITION_TOLERANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize PID controller
        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(POSITION_TOLERANCE);

        // Initialize motor as DcMotorEx for better control
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        // Configure motor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // We'll control it with PID
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial target position
        int targetPosition = ARM_DOWN_POSITION;
        controller.setSetPoint(targetPosition);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            // Get current position
            int currentPosition = armMotor.getCurrentPosition();

            // Update target position based on gamepad input
            if (gamepad2.a) {
                targetPosition = ARM_UP_POSITION;
                controller.setSetPoint(targetPosition);
            } else if (gamepad2.b) {
                targetPosition = ARM_DOWN_POSITION;
                controller.setSetPoint(targetPosition);
            } else if (gamepad2.y) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPosition = 0;
                controller.setSetPoint(targetPosition);
            } else if (gamepad2.x) {
                targetPosition = 7000;
                controller.setSetPoint(targetPosition);
            } else if (gamepad2.dpad_up) {
                targetPosition = currentPosition + POSITION_CHANGE;
                controller.setSetPoint(targetPosition);
            } else if (gamepad2.dpad_down) {
                targetPosition = currentPosition - POSITION_CHANGE;
                controller.setSetPoint(targetPosition);
            }

            // Calculate PID output
            double pid = controller.calculate(currentPosition);

            // Apply PID output to motor
            // Clamp the power between -1 and 1
            double power = Math.min(Math.max(pid, -0.7), 0.7); // Limiting max power to 70% for safety
            armMotor.setPower(power);

            // Display telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", controller.getPositionError());
            telemetry.addData("PID Output", pid);
            telemetry.addData("Motor Power", power);
            telemetry.addData("At Setpoint", controller.atSetPoint());
            telemetry.update();
        }
    }
}
