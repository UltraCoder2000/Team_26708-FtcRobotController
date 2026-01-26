package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "VisionAuto", preselectTeleOp = "DECODE TeleOp")
public class VisionAuto extends OpMode {

    enum AutoState {
        DRIVE_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        DRIVE_PICKUP_1,
        PICKUP_1,
        DRIVE_TO_SHOOT_1,
        SHOOT_1,
        DRIVE_PICKUP_2,
        PICKUP_2,
        DRIVE_TO_SHOOT_2,
        SHOOT_2,
        DRIVE_PICKUP_3,
        PICKUP_3,
        DRIVE_TO_SHOOT_3,
        SHOOT_3,
        PARK,
        IDLE
    }

    private AutoState state = AutoState.DRIVE_TO_SHOOT_PRELOAD;

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    private Limelight3A LM;

    private long stateStartTime;

    private static final double DISTANCE_BUFFER = 0.15;
    private static final double KV = 850;
    private static final double KSTATIC = 900;
    private static final double VELOCITY_TOLERANCE = 75;

    private double alliance = 1; // default blue side

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");

        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
        transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

        shooters = new DcMotorEx[]{leftShooter, rightShooter};

        PIDFCoefficients coeffs = new PIDFCoefficients(15, 0.05, 2.5, 13.2);
        for (DcMotorEx m : shooters) {
            m.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);

        LM = hardwareMap.get(Limelight3A.class, "LM");

        LM.pipelineSwitch(0);
        LM.start();
        stateStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        switch (state) {
            case DRIVE_TO_SHOOT_PRELOAD:
                driveForward(0.5);
                if (timeInState() > 2000) { // 2 seconds to shooting pos (example)
                    stopDrive();
                    stateTransition(AutoState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shootRings();
                if (timeInState() > 1500) { // 1.5 sec shooting
                    stopShooting();
                    stateTransition(AutoState.DRIVE_PICKUP_1);
                }
                break;

            case DRIVE_PICKUP_1:
                driveBackward(0.6);
                intakeMotor.setPower(1);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.PICKUP_1);
                }
                break;

            case PICKUP_1:
                // short pause to ensure intake
                if (timeInState() > 800) {
                    stateTransition(AutoState.DRIVE_TO_SHOOT_1);
                }
                break;

            case DRIVE_TO_SHOOT_1:
                driveForward(0.6);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.SHOOT_1);
                }
                break;

            case SHOOT_1:
                shootRings();
                if (timeInState() > 1500) {
                    stopShooting();
                    stateTransition(AutoState.DRIVE_PICKUP_2);
                }
                break;

            case DRIVE_PICKUP_2:
                driveBackward(0.6);
                intakeMotor.setPower(1);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.PICKUP_2);
                }
                break;

            case PICKUP_2:
                if (timeInState() > 800) {
                    stateTransition(AutoState.DRIVE_TO_SHOOT_2);
                }
                break;

            case DRIVE_TO_SHOOT_2:
                driveForward(0.6);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.SHOOT_2);
                }
                break;

            case SHOOT_2:
                shootRings();
                if (timeInState() > 1500) {
                    stopShooting();
                    stateTransition(AutoState.DRIVE_PICKUP_3);
                }
                break;

            case DRIVE_PICKUP_3:
                driveBackward(0.6);
                intakeMotor.setPower(1);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.PICKUP_3);
                }
                break;

            case PICKUP_3:
                if (timeInState() > 800) {
                    stateTransition(AutoState.DRIVE_TO_SHOOT_3);
                }
                break;

            case DRIVE_TO_SHOOT_3:
                driveForward(0.6);
                if (timeInState() > 1500) {
                    stopDrive();
                    stateTransition(AutoState.SHOOT_3);
                }
                break;

            case SHOOT_3:
                shootRings();
                if (timeInState() > 1500) {
                    stopShooting();
                    stateTransition(AutoState.PARK);
                }
                break;

            case PARK:
                driveForward(0.5);
                if (timeInState() > 1000) { // park time
                    stopDrive();
                    stateTransition(AutoState.IDLE);
                }
                break;

            case IDLE:
                stopAll();
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("Shooter Vel LF", leftShooter.getVelocity());
        telemetry.addData("Shooter Vel RF", rightShooter.getVelocity());
        telemetry.update();
    }

    /* ----------------- DRIVE ----------------- */

    private void driveForward(double power) {
        leftFront.setPower(power * alliance);
        rightFront.setPower(power * alliance);
        leftBack.setPower(power * alliance);
        rightBack.setPower(power * alliance);
    }

    private void driveBackward(double power) {
        driveForward(-power);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void stopAll() {
        stopDrive();
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    /* ----------------- SHOOTER ----------------- */

    private void shootRings() {
        double targetVel = getShooterVelocity();
        leftShooter.setVelocity(targetVel);
        rightShooter.setVelocity(targetVel);
        transferMotor.setPower(1);
    }

    private void stopShooting() {
        transferMotor.setPower(0);
        intakeMotor.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    private double getShooterVelocity() {
        LLResult result = LM.getLatestResult();
        if (result == null || !result.isValid()) return 1550;

        double ty = result.getTy();

        // Camera / field constants (TUNE THESE)
        final double CAMERA_HEIGHT = 8.0;      // inches
        final double TARGET_HEIGHT = 40.0;     // inches (AprilTag center)
        final double CAMERA_ANGLE = 25.0;      // degrees (mount angle)

        // Distance calculation using ty
        double distance =
                (TARGET_HEIGHT - CAMERA_HEIGHT) /
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));

        // Shoot slightly farther than measured distance
        double effectiveDistance = distance + DISTANCE_BUFFER;

        return (KV * effectiveDistance) + KSTATIC;
    }

    /* ----------------- UTIL ----------------- */

    private void stateTransition(AutoState next) {
        state = next;
        stateStartTime = System.currentTimeMillis();
    }

    private long timeInState() {
        return System.currentTimeMillis() - stateStartTime;
    }
}
