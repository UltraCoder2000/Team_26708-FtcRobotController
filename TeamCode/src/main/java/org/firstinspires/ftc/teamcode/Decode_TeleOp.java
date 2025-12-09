package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DecodeTeleOp")
public class DecodeTeleOp extends LinearOpMode {
    
    // Hardware components
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx leftShooter, rightShooter, intakeMotor;
    private CRServo transferServo;

    // Timing and control variables
    private long previousLoopEnd = 0;
    private double startingTransferSpeed;
    private double startingShooterSpeed;
    
    // Control speed coefficients
    private static final double SLOW_MODE_COEFF = 1050.0;
    private static final double FAST_MODE_COEFF = 2800.0;
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
        transferServo = hardwareMap.get(CRServo.class, "TR");

        DcMotor[] driveMotors = {leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        transferServo.setDirection(DcMotor.Direction.REVERSE);


    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;   // forward/back
        double strafe = gamepad1.left_stick_x;   // lef t/right
        double turn = gamepad1.right_stick_x;    // rotation

        // Compute raw motor values (without coefficient)
        double rawLB = drive + turn - strafe;
        double rawRB = drive - turn + strafe;
        double rawLF = drive + turn + strafe;
        double rawRF = drive - turn - strafe;

        // Find maximum absolute raw value
        double maxRaw = Math.max(
                Math.max(Math.abs(rawLB), Math.abs(rawRB)),
                Math.max(Math.abs(rawLF), Math.abs(rawRF))
        );

        double lbN = rawLB, rbN = rawRB, lfN = rawLF, rfN = rawRF;

        // Compute average of absolute reduced values and square it for smoother scaling
        double avgAbs = (Math.abs(lbN) + Math.abs(rbN) + Math.abs(lfN) + Math.abs(rfN)) / 4.0;
        double scale = avgAbs * avgAbs; // squared average magnitude

        // Choose coefficient based on stick-button (fast when pressed)
        double coeff = (gamepad1.left_stick_button || gamepad1.right_stick_button)
                        ? FAST_MODE_COEFF
                        : SLOW_MODE_COEFF;

        // Final velocities (apply scale and coefficient)
        double velLB = lbN * scale * coeff;
        double velRB = rbN * scale * coeff;
        double velLF = lfN * scale * coeff;
        double velRF = rfN * scale * coeff;

        leftBackDrive.setVelocity(velLB);
        rightBackDrive.setVelocity(velRB);
        leftFrontDrive.setVelocity(velLF);
        rightFrontDrive.setVelocity(velRF);

        telemetry.addData("MaxRaw", "%.3f", maxRaw);
        telemetry.addData("AvgAbs", "%.3f", avgAbs);
        telemetry.addData("Scale", "%.3f", scale);
    }
    
    private void intake() {
        double intakePower = -gamepad2.left_stick_y;
        intakeMotor.setPower(intakePower);

        telemetry.addData("Intake Power", intakePower);
        telemetry.update();
    }
    
    private void transfer() {
        if (gamepad2.left_bumper) {
            startingTransferSpeed = -1.0;
        } else {
            startingTransferSpeed = 0.0;
        }
        
        transferServo.setPower(startingTransferSpeed + gamepad2.left_trigger);
    }
    
    private void shoot() {
        if (gamepad2.right_bumper) {
            startingShooterSpeed = -TOP_SHOOTER_SPEED;
        } else {
            startingShooterSpeed = 0.0;
        }
        
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotor motor : shooters) {
            motor.setPower(startingShooterSpeed + TOP_SHOOTER_SPEED * gamepad2.right_trigger);
        }
    }

    private void updateTelemetry() {
        long loopTime = System.currentTimeMillis() - previousLoopEnd;
        previousLoopEnd = System.currentTimeMillis();
        telemetry.addLine("Loop Time = " + loopTime + " ms");
        telemetry.addLine("Drive Mode: " + 
            ((gamepad1.left_stick_button || gamepad1.right_stick_button) ? "FAST (2800)" : "SLOW (1050)"));
        telemetry.update();
    }
}

    


