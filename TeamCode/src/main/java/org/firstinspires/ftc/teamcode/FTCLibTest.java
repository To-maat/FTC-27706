package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class FTCLibTest extends LinearOpMode {
    private PIDController controller;

    // PID Coefficients - now configurable through dashboard
    public static double kP = 0.05;
    public static double kI = 0.1;
    public static double kD = 0.0001;

    // Position constants - now configurable through dashboard
    public static int ARM_UP_POSITION = 3000;
    public static int ARM_DOWN_POSITION = 0;
    public static int POSITION_CHANGE = 200;
    public static int CUSTOM_POSITION = 1500; // New configurable position

    // Tolerance for position control
    public static double POSITION_TOLERANCE = 20;

    // Motor power limit
    public static double MAX_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
            // Update PID coefficients from dashboard
            controller.setPID(kP, kI, kD);
            controller.setTolerance(POSITION_TOLERANCE);

            // Get current position
            int currentPosition = armMotor.getCurrentPosition();

            // Update target position based on gamepad input
            if (gamepad2.a) {
                targetPosition = ARM_UP_POSITION;
            } else if (gamepad2.b) {
                targetPosition = ARM_DOWN_POSITION;
            } else if (gamepad2.y) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPosition = 0;
            } else if (gamepad2.x) {
                targetPosition = CUSTOM_POSITION;
            } else if (gamepad2.dpad_up) {
                targetPosition = currentPosition + POSITION_CHANGE;
            } else if (gamepad2.dpad_down) {
                targetPosition = currentPosition - POSITION_CHANGE;
            }

            // Update setpoint
            controller.setSetPoint(targetPosition);

            // Calculate PID output
            double pid = controller.calculate(currentPosition);

            // Apply PID output to motor with configurable power limit
            double power = Math.min(Math.max(pid, -MAX_POWER), MAX_POWER);
            armMotor.setPower(power);

            // Display telemetry
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Position Error", controller.getPositionError());
            telemetry.addData("PID Output", pid);
            telemetry.addData("Motor Power", power);
            telemetry.addData("At Setpoint", controller.atSetPoint());
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.update();
        }
    }
}
