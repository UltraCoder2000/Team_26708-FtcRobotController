package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "VisionTeleopDistanceShooter")
public class VisionTeleopDistanceShooter extends LinearOpMode {

    // Drive motors
    private DcMotorEx LB, LF, RB, RF;

    // Mechanisms
    private DcMotorEx LS, RS, IN, TR;

    // Limelight
    private Limelight3A limelight;

    // PID constants for heading alignment
    private static final double kP = 0.025;
    private static final double kI = 0.0;
    private static final double kD = 0.002;

    private double errorSum = 0;
    private double lastError = 0;
    private long lastPidTime = 0;

    // Drive constants
    private static final double SLOW_MODE_COEFF = 1050.0;
    private static final double FAST_MODE_COEFF = 2800.0;
    private static final double MAX_TURN_OUTPUT = 0.4;

    // Shooter constants
    private static final double MAX_SHOOTER_SPEED = 2800.0; // max motor velocity
    private static final double MIN_SHOOTER_SPEED = 1800.0; // close-range RPM
    private static final double BASE_SHOOTER_SPEED = 2200.0; // mid-range RPM

    private long lastLoop = 0;

    // Camera calibration constants
    private static final double CAMERA_HEIGHT = 0.30; // meters
    private static final double TARGET_HEIGHT = 0.80; // meters (center of target)
    private static final double CAMERA_ANGLE = 20.0;  // degrees, mounting tilt

    @Override
    public void runOpMode() {

        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            intake();
            transfer();
            shoot();
            telemetryLoop();
        }
    }

    private void initHardware() {

        // Drive motors
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");

        // Mechanisms
        LS = hardwareMap.get(DcMotorEx.class, "LS");
        RS = hardwareMap.get(DcMotorEx.class, "RS");
        IN = hardwareMap.get(DcMotorEx.class, "IN");
        TR = hardwareMap.get(DcMotorEx.class, "TR");

        DcMotorEx[] drives = {LB, LF, RB, RF};
        for (DcMotorEx m : drives) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        LS.setDirection(DcMotor.Direction.REVERSE);

        // Shooter PID
        PIDFCoefficients shooterPID = new PIDFCoefficients(15, 0.2, 2.5, 13.2);
        LS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
        RS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }

    private void drive() {

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn;

        boolean align = gamepad1.a;
        double speedScale = 1.0;

        LLResult result = limelight.getLatestResult();

        if (align && result != null && result.isValid()) {

            double tx = result.getTx();

            // PID turn
            long now = System.currentTimeMillis();
            double dt = (now - lastPidTime) / 1000.0;
            lastPidTime = now;

            if (dt > 0) {
                double error = tx;
                errorSum += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;

                turn = (kP * error) + (kI * errorSum) + (kD * derivative);
                turn = Math.max(-MAX_TURN_OUTPUT, Math.min(MAX_TURN_OUTPUT, turn));
            } else {
                turn = 0;
            }

            speedScale = 1.0;

            telemetry.addLine("AprilTag ALIGN ACTIVE");
            telemetry.addData("tx", tx);

        } else {
            turn = gamepad1.right_stick_x;
            errorSum = 0;
            lastError = 0;
            lastPidTime = System.currentTimeMillis();
        }

        // Mecanum math
        double LBp = drive + turn - strafe;
        double RBp = drive - turn + strafe;
        double LFp = drive + turn + strafe;
        double RFp = drive - turn - strafe;

        double maxMag = Math.max(
                Math.max(Math.abs(LBp), Math.abs(RBp)),
                Math.max(Math.abs(LFp), Math.abs(RFp))
        );

        if (maxMag > 1.0) {
            LBp /= maxMag;
            RBp /= maxMag;
            LFp /= maxMag;
            RFp /= maxMag;
        }

        double baseSpeed =
                (gamepad1.left_stick_button || gamepad1.right_stick_button)
                        ? FAST_MODE_COEFF
                        : SLOW_MODE_COEFF;

        LB.setVelocity(LBp * speedScale * baseSpeed);
        RB.setVelocity(RBp * speedScale * baseSpeed);
        LF.setVelocity(LFp * speedScale * baseSpeed);
        RF.setVelocity(RFp * speedScale * baseSpeed);
    }

    private void intake() {
        IN.setPower(-gamepad2.left_stick_y);
    }

    private void transfer() {
        double power = gamepad2.left_trigger;
        if (gamepad2.left_bumper) power = -1.0;
        TR.setPower(power);
    }

    private void shoot() {

        LLResult result = limelight.getLatestResult();

        double shooterTarget = BASE_SHOOTER_SPEED;

        if (result != null && result.isValid()) {

            double ty = result.getTy(); // vertical angle to target

            // Estimate distance using simple trig
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) /
                    Math.tan(Math.toRadians(CAMERA_ANGLE + ty));

            // Map distance to shooter RPM linearly
            // Clamp to min/max
            shooterTarget = BASE_SHOOTER_SPEED + distance * 600; // adjust 600 empirically
            if (shooterTarget > MAX_SHOOTER_SPEED) shooterTarget = MAX_SHOOTER_SPEED;
            if (shooterTarget < MIN_SHOOTER_SPEED) shooterTarget = MIN_SHOOTER_SPEED;

            telemetry.addData("ty", ty);
            telemetry.addData("Distance (m)", distance);
            telemetry.addData("Shooter RPM", shooterTarget);
        }

        // Allow manual control override
        if (gamepad2.right_bumper) {
            shooterTarget += BASE_SHOOTER_SPEED * gamepad2.right_trigger;
            if (shooterTarget > MAX_SHOOTER_SPEED) shooterTarget = MAX_SHOOTER_SPEED;
        }

        LS.setVelocity(shooterTarget);
        RS.setVelocity(shooterTarget);
    }

    private void telemetryLoop() {
        long now = System.currentTimeMillis();
        telemetry.addData("Loop ms", now - lastLoop);
        lastLoop = now;
        telemetry.update();
    }
}
