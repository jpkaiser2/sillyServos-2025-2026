package org.firstinspires.ftc.teamcode;

// Import statements - these bring in the FTC SDK classes we need

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// @TeleOp annotation tells the FTC app that this is a driver-controlled program
// It will appear in the TeleOp menu on the Driver Station
@TeleOp
public class TeleOpMain extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Turret motor
    private DcMotor turret;

    // Intake motor
    private DcMotor intake;

    // Turret angle servo
    private Servo turretAngle;
    private double turretAnglePos = 0.5; // initial position

    // Flywheel launcher
    private DcMotorEx flywheel;
    private boolean flywheelEnabled = false;

    // Fixed target velocity in ticks per second
    private static final double FLYWHEEL_FIXED_TPS = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Map turret angle servo
        turretAngle = hardwareMap.get(Servo.class, "turretAngle");

        // Map flywheel motor
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");


        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Turret direction
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake direction
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Flywheel direction
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize turret angle servo position
        turretAngle.setPosition(turretAnglePos);

        // Flywheel uses FLOAT to keep momentum when power is cut
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set encoder mode for velocity control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start button
        waitForStart();

        // Main teleop loop
        while (opModeIsActive()) {
            // Read gamepad sticks
            double y = -gamepad1.left_stick_y;   // forward/back (invert so up is +)
            double x = gamepad1.left_stick_x;    // strafe left/right
            double rx = gamepad1.right_stick_x;  // rotation

            // Turret control: map to D-pad left/right
            double turretInput = 0.0;
            if (gamepad1.dpad_left) turretInput -= 1.0;
            if (gamepad1.dpad_right) turretInput += 1.0;
            double turretScale;

            if (gamepad1.left_bumper) {
                turretScale = 0.3; // slower when holding LB
            } else {
                turretScale = 0.6; // default moderate speed
            }

            // Apply small deadband to avoid drift
            if (Math.abs(turretInput) < 0.05) turretInput = 0.0;

            // Turret angle servo: D-pad up/down to adjust
            double angleStep;
            if (gamepad1.left_bumper) {
                angleStep = 0.002; // finer when LB
            } else {
                angleStep = 0.01;
            }

            if (gamepad1.dpad_up) turretAnglePos = Math.min(1.0, turretAnglePos + angleStep);
            if (gamepad1.dpad_down) turretAnglePos = Math.max(0.0, turretAnglePos - angleStep);
            turretAngle.setPosition(turretAnglePos);

            // Intake control: gamepad1 triggers
            // Left trigger = intake in, Right trigger = out (reverse). Combine as right - left so positive = out
            double intakeIn = gamepad1.left_trigger;   // 0..1
            double intakeOut = gamepad1.right_trigger; // 0..1
            double intakeCmd = intakeIn - intakeOut;   // positive = in, negative = out

            // Optional slow/precision mode with gamepad1 right bumper (same as drive slow)
            double intakeScale;
            if (gamepad1.right_bumper) {
                intakeScale = 0.5;
            } else {
                intakeScale = 1.0;
            }
            // Deadband
            if (Math.abs(intakeCmd) < 0.05) intakeCmd = 0.0;

            // Flywheel control: gamepad1 A to spin up, B to stop
            if (gamepad1.a) flywheelEnabled = true;
            if (gamepad1.b) flywheelEnabled = false;

            // Optional slow mode with right bumper
            double speedScale;
            if (gamepad1.right_bumper) {
                speedScale = 0.4;
            } else {
                speedScale = 1.0;
            }

            // Compute raw wheel powers for mecanum
            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            // Normalize so no value exceeds 1.0
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl = fl / max * speedScale;
            fr = fr / max * speedScale;
            bl = bl / max * speedScale;
            br = br / max * speedScale;

            // Set powers
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);
            turret.setPower(turretInput * turretScale);
            intake.setPower(intakeCmd * intakeScale);

            // Apply flywheel velocity or stop
            if (flywheelEnabled) {
                flywheel.setPower(1.0);
            } else {
                flywheel.setPower(0.0);
            }

            // Telemetry for debugging
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("FL", fl);
            telemetry.addData("FR", fr);
            telemetry.addData("BL", bl);
            telemetry.addData("BR", br);
            telemetry.addData("TurretIn (gp1 D-pad)", turretInput);
            telemetry.addData("TurretPower", turretInput * turretScale);
            telemetry.addData("IntakeCmd (gp1)", intakeCmd);
            telemetry.addData("IntakePower", intakeCmd * intakeScale);
            telemetry.addData("FlywheelEnabled", flywheelEnabled);
            telemetry.addData("FlywheelTargetTPS", FLYWHEEL_FIXED_TPS);
            telemetry.addData("TurretAnglePos", turretAnglePos);
            telemetry.update();
        }
    }
}

/*
Robot Controls (gamepad1)
-------------------------

Drive (Mecanum):
- Left stick Y: Forward/Backward
- Left stick X: Strafe Left/Right
- Right stick X: Rotate Robot
- Right bumper: Drive slow mode (also applies to intake)

Turret Rotation:
- D-pad Left: Rotate turret left
- D-pad Right: Rotate turret right
- Left bumper: Turret slow mode

Turret Angle (Servo):
- D-pad Up: Increase turret angle
- D-pad Down: Decrease turret angle
- Left bumper: Fine adjustment for angle

Intake:
- Left trigger: Intake in
- Right trigger: Intake out (reverse)
- Right bumper: Intake slow mode

Flywheel Launcher:
- A: Spin up flywheel to fixed high speed
- B: Stop flywheel
*/
