package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "DECODE TeleOp")
public class DecodeTeleOp extends LinearOpMode {
    // Motors
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    // Constants
    final double MOUNT_HEIGHT = 0.26035;
    final double TARGET_HEIGHT = 0.74930;
    final double MOUNT_ANGLE = 16.0;
    private final double KP = 0.075;
    private final double KF = 0.015;
    private final double MAX_TURN_OUTPUT = 0.75;

    // Variables
    private double horizontalDistance = -1;
    private boolean autoAimEnabled = false;
    private double targetShooterVelocity = 0;
    private double aimedShooterSpeed = 0;
    private enum ShooterMode {
        AUTO,
        BACK,
        ZERO,
        CLOSE,
        MID,
        FAR
    }
    ShooterMode shooterMode = ShooterMode.ZERO;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "Initialized, waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            limelightLogic();
            drive();
            intakeAndTransfer();
            shooterLogic();
            updateTelemetry();
        }
    }

    private void limelightLogic() {
        // Updates Limelight values
        LLResult result = limelight.getLatestResult();

        // If result valid, calculates position of bot and distance to goal
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double totalAngleRadians = Math.toRadians(MOUNT_ANGLE + ty);
            horizontalDistance = (TARGET_HEIGHT - MOUNT_HEIGHT) / Math.tan(totalAngleRadians);
        } else {
            horizontalDistance = -1; // Placeholder value as no valid result
        }

        // Redundancy in case of Autonomous program failing
        if (gamepad1.dpad_left) {
            DataPasser.currentAlliance = DataPasser.Alliance.BLUE;
            imu.resetYaw();
            limelight.pipelineSwitch(0);
        }
        if (gamepad1.dpad_right) {
            DataPasser.currentAlliance = DataPasser.Alliance.RED;
            imu.resetYaw();
            limelight.pipelineSwitch(1);
        }

        // Disables chassis auto-aim if gamepad 1 touches sticks
        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
            autoAimEnabled = false;
        }

        // Overrides above case
        if (gamepad2.right_trigger_pressed) {
            autoAimEnabled = true;
        }
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        LLResult result = limelight.getLatestResult();

        if (autoAimEnabled && result != null && result.isValid()) {
            double tx = result.getTx();
            turn = tx * KP + Math.signum(tx) * KF;
            turn = Math.max(-MAX_TURN_OUTPUT, Math.min(MAX_TURN_OUTPUT, turn));
        }

        double LB = drive + turn - strafe;
        double RB = drive - turn + strafe;
        double LF = drive + turn + strafe;
        double RF = drive - turn - strafe;

        double coeff = (gamepad1.left_stick_button || gamepad1.right_stick_button) ? 2800.0 : 1200.0;

        leftBackDrive.setVelocity(LB * coeff);
        rightBackDrive.setVelocity(RB * coeff);
        leftFrontDrive.setVelocity(LF * coeff);
        rightFrontDrive.setVelocity(RF * coeff);
    }

    private void intakeAndTransfer() {
        intakeMotor.setPower(-gamepad2.left_stick_y);
        if (gamepad2.left_bumper) {
            transferMotor.setPower(-1.0);
        } else {
            transferMotor.setPower(0.6 * gamepad2.left_trigger - 0.2);
        }
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }

    private void shooterLogic() {
        if (gamepad2.right_bumper)       shooterMode = ShooterMode.AUTO;
        if (gamepad2.right_stick_button) shooterMode = ShooterMode.BACK;
        if (gamepad2.a)                  shooterMode = ShooterMode.ZERO;
        if (gamepad2.b)                  shooterMode = ShooterMode.CLOSE;
        if (gamepad2.x)                  shooterMode = ShooterMode.MID;
        if (gamepad2.y)                  shooterMode = ShooterMode.FAR;

        if (horizontalDistance != -1)         aimedShooterSpeed = (0.00333057 * Math.pow(33.55221, horizontalDistance)) + (191.64674 * horizontalDistance) + 971.0142;
        if (shooterMode == ShooterMode.AUTO)  targetShooterVelocity = aimedShooterSpeed;
        if (shooterMode == ShooterMode.BACK)  targetShooterVelocity = -500;
        if (shooterMode == ShooterMode.ZERO)  targetShooterVelocity = 0;
        if (shooterMode == ShooterMode.CLOSE) targetShooterVelocity = 1200;
        if (shooterMode == ShooterMode.MID)   targetShooterVelocity = 1350;
        if (shooterMode == ShooterMode.FAR)   targetShooterVelocity = 1700;
        if (targetShooterVelocity > 2000)     targetShooterVelocity = 2000;

        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx shooter : shooters) {
            shooter.setVelocity(targetShooterVelocity * 7/15);
        }
    }

    private void initHardware() {
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "LB");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LF");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "RB");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "RF");

        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
        transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

        DcMotor[] drives = {leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : drives) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx[] loaders = {intakeMotor, transferMotor};
        for (DcMotorEx motor : loaders) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        PIDFCoefficients coeffs = new PIDFCoefficients(45.0, 0.02, 2.5, 13.2);
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        }

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "LM");
        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }
        limelight.start();
    }

    private void updateTelemetry() {
        telemetry.addData("Target Found", (horizontalDistance == -1) ? "NO" : "YES");
        telemetry.addData("Distance to Target", "%.2f", horizontalDistance);
        telemetry.addData("Shooter Velocity", targetShooterVelocity);
        telemetry.addData("Actual Left Velocity", leftShooter.getVelocity() * 15/7);
        telemetry.addData("Actual Right Velocity", rightShooter.getVelocity() * 15/7);
        telemetry.addData("Auto-Aim Active", autoAimEnabled);
        telemetry.addLine("Drive Mode: " + ((gamepad1.left_stick_button || gamepad1.right_stick_button) ? "FAST (2800)" : "SLOW (1050)"));
        telemetry.update();
    }
}