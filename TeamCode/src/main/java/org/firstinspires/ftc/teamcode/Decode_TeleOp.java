package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "DecodeTeleOp")
public class DecodeTeleOp extends LinearOpMode {
    
    // Hardware components
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive,
                      leftShooter,   rightShooter,   intakeMotor,    transferMotor;

    // Timing and control variables
    private long previousLoopEnd = 0;
    private double startingTransferSpeed;
    private double startingShooterSpeed;
    
    // Control speed coefficients
    private static final double SLOW_MODE_COEFF = 1050.0;
    private static final double FAST_MODE_COEFF = 2800.0;
    private static final double MAX_SPEED_COEFF = 2800.0;
    private static final double TOP_SHOOTER_SPEED = 0.3;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            drive();
            intake();
            transfer();
            shoot();
            updateTelemetry();
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

        DcMotor[] driveMotors = {leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        PIDFCoefficients coeffs = new PIDFCoefficients(15.0, 0.2, 2.5, 13.2);
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setPIDFCoefficients(coeffs);
        }
        
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;   // forward/back
        double strafe = gamepad1.left_stick_x;   // left/right
        double turn = gamepad1.right_stick_x;    // rotation

        // Compute raw motor power values
        double LB = drive + turn - strafe;
        double RB = drive - turn + strafe;
        double LF = drive + turn + strafe;
        double RF = drive - turn - strafe;

        // For smoother scaling, square the average of the absolute values of the raw numbers above
        double avgAbs = (Math.abs(LB) + Math.abs(RB) + Math.abs(LF) + Math.abs(RF)) / 4.0;
        double scale = avgAbs * avgAbs; // squared average magnitude

        // Choose drive speed coefficient based on left stick press
        double coeff = (gamepad1.left_stick_button || gamepad1.right_stick_button)
                        ? FAST_MODE_COEFF
                        : SLOW_MODE_COEFF;

        // Final velocities (apply scale and coefficient)
        double velLB = LB * scale * coeff;
        double velRB = RB * scale * coeff;
        double velLF = LF * scale * coeff;
        double velRF = RF * scale * coeff;

        // Set motor velocities
        leftBackDrive.setVelocity(velLB);
        rightBackDrive.setVelocity(velRB);
        leftFrontDrive.setVelocity(velLF);
        rightFrontDrive.setVelocity(velRF);

        // Send important values to telemetry
        telemetry.addData("Scale", "%.3f", scale);
        telemetry.addData("LB Intended Velocity", velLB);
        telemetry.addData("RB Intended Velocity", velRB);
        telemetry.addData("LF Intended Velocity", velLF);
        telemetry.addData("RF Intended Velocity", velRF);
        telemetry.addLine("Drive Mode: " + 
            ((gamepad1.left_stick_button || gamepad1.right_stick_button) ? "FAST (2800)" : "SLOW (1050)"));
    }
    
    private void intake() {
        double intakePower = -gamepad2.left_stick_y;
        intakeMotor.setPower(intakePower);
        telemetry.addData("Intake Power", intakePower);
    }
    
    private void transfer() {
        if (gamepad2.left_bumper) {
            startingTransferSpeed = -1.0;
        } else {
            startingTransferSpeed = 0.0;
        }
        
        transferMotor.setPower(startingTransferSpeed + gamepad2.left_trigger);
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }
    
    private void shoot() {
        if (gamepad2.right_bumper) {
            startingShooterSpeed = -TOP_SHOOTER_SPEED;
        } else {
            startingShooterSpeed = 0.0;
        }
        
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setVelocity(MAX_SPEED_COEFF * (startingShooterSpeed + TOP_SHOOTER_SPEED * gamepad2.right_trigger));
        }
    }

    private void updateTelemetry() {
        long loopTime = System.currentTimeMillis() - previousLoopEnd;
        previousLoopEnd = System.currentTimeMillis();
        telemetry.addLine("Loop Time = " + loopTime + " ms");
        telemetry.update();
    }
}

