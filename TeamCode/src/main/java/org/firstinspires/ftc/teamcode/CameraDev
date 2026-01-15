package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "Decode TeleOp")
public class DecodeTeleOp extends LinearOpMode {

    // Drive motors
    private DcMotorEx LB, LF, RB, RF;

    // Mechanisms
    private DcMotorEx LS, RS, IN, TR;

    // Limelight
    private NetworkTable limelight;

    // PID constants (tune kP first)
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

    // Shooter
    private static final double MAX_SHOOTER_SPEED = 2800.0;
    private static final double TOP_SHOOTER_SPEED = 0.25;

    private long lastLoop = 0;

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
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");

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

        PIDFCoefficients shooterPID = new PIDFCoefficients(15, 0.2, 2.5, 13.2);
        LS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
        RS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);

        limelight = NetworkTableInstance.getDefault().getTable("limelight");

        // Force Limelight into known state
        limelight.getEntry("ledMode").setNumber(3);     // LEDs ON
        limelight.getEntry("pipeline").setNumber(0);   // AprilTag pipeline
    }

    private void drive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn;

        boolean align = gamepad1.a;
        double tv = limelight.getEntry("tv").getDouble(0);

        if (align && tv == 1) {
            double tx = limelight.getEntry("tx").getDouble(0);

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

            telemetry.addLine("AprilTag ALIGN ACTIVE");
            telemetry.addData("tx", tx);
        } else {
            turn = gamepad1.right_stick_x;
            errorSum = 0;
            lastError = 0;
            lastPidTime = System.currentTimeMillis();
        }

        double LBp = drive + turn - strafe;
        double RBp = drive - turn + strafe;
        double LFp = drive + turn + strafe;
        double RFp = drive - turn - strafe;

        double avg = (Math.abs(LBp) + Math.abs(RBp) + Math.abs(LFp) + Math.abs(RFp)) / 4.0;
        double scale = avg * avg;

        double speed = (gamepad1.left_stick_button || gamepad1.right_stick_button)
                ? FAST_MODE_COEFF
                : SLOW_MODE_COEFF;

        LB.setVelocity(LBp * scale * speed);
        RB.setVelocity(RBp * scale * speed);
        LF.setVelocity(LFp * scale * speed);
        RF.setVelocity(RFp * scale * speed);
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
        double target = TOP_SHOOTER_SPEED;
        if (gamepad2.right_bumper) {
            target += TOP_SHOOTER_SPEED * gamepad2.right_trigger;
        }
        double velocity = MAX_SHOOTER_SPEED * target;
        LS.setVelocity(velocity);
        RS.setVelocity(velocity);
    }

    private void telemetryLoop() {
        long now = System.currentTimeMillis();
        telemetry.addData("Loop ms", now - lastLoop);
        lastLoop = now;
        telemetry.update();
    }
}
