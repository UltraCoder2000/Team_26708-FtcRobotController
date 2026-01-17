//helo
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardPIDFCoefficients;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "Decode TeleOp")
public class DecodeTeleOp extends LinearOpMode {

    // Drive motors
    private DcMotorEx LB, LF, RB, RF;
    ware.
    // Mechanisms
    private DcMotorEx LS, RS, IN, TR;

    // Limelight
    private NetworkTable limelight;

    // Alignment PID
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
    private static final double MIN_SHOOTER_RPM = 1800.0;
    private static final double MAX_SHOOTER_RPM = 2800.0;

    private static final double MIN_SHOOT_DISTANCE = 0.5; // meters
    private static final double MAX_SHOOT_DISTANCE = 2.0; // meters

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
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("pipeline").setNumber(0);
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

        double max = Math.max(Math.max(Math.abs(LBp), Math.abs(RBp)),
                Math.max(Math.abs(LFp), Math.abs(RFp)));
        if (max > 1.0) {
            LBp /= max;
            RBp /= max;
            LFp /= max;
            RFp /= max;
        }

        double speed = (gamepad1.left_stick_button || gamepad1.right_stick_button)
                ? FAST_MODE_COEFF
                : SLOW_MODE_COEFF;

        LB.setVelocity(LBp * speed);
        RB.setVelocity(RBp * speed);
        LF.setVelocity(LFp * speed);
        RF.setVelocity(RFp * speed);
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
        double tv = limelight.getEntry("tv").getDouble(0);
        double targetRPM = 0;

        if (gamepad2.right_bumper && tv == 1) {
            double[] pose = limelight.getEntry("botpose_targetspace")
                    .getDoubleArray(new double[6]);

            if (pose.length == 6) {
                double distance = pose[2];

                // Linear interpolation
                double t = (distance - MIN_SHOOT_DISTANCE) /
                        (MAX_SHOOT_DISTANCE - MIN_SHOOT_DISTANCE);

                t = Math.max(0.0, Math.min(1.0, t));
                targetRPM = MIN_SHOOTER_RPM +
                        t * (MAX_SHOOTER_RPM - MIN_SHOOTER_RPM);

                telemetry.addData("Shooter Distance (m)", distance);
            }
        }

        // Manual fallback (no tag or bumper not pressed)
        if (!gamepad2.right_bumper) {
            targetRPM = 0;
        }

        LS.setVelocity(targetRPM);
        RS.setVelocity(targetRPM);

        telemetry.addData("Shooter RPM", targetRPM);
    }

    private void telemetryLoop() {
        long now = System.currentTimeMillis();
        telemetry.addData("Loop ms", now - lastLoop);
        lastLoop = now;
        telemetry.update();
    }

}
