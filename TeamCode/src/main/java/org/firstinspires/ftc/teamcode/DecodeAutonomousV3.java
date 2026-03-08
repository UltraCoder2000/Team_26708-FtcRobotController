package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// ══════════════════════════════════════════════════════════════════════════════
//  DecodeAutonomous20s_v4
//
//  SEQUENCE:
//    1.  Shoot preload  (at shootPose)
//    2.  Intake B2
//    3.  Shoot
//    4.  Intake from Gate  (drive to x=56, y=36, heading=41° — hold lever, balls roll in)
//    5.  Shoot
//    6.  Intake from Gate  (second trip — same pose)
//    7.  Shoot
//    8.  Intake B1
//    9.  Shoot
//    10. Intake B3
//    11. Shoot  → IDLE (20s hard cap)
// ══════════════════════════════════════════════════════════════════════════════

@Autonomous(name = "DECODE Autonomous 20s v4", preselectTeleOp = "Decode TeleOp LumoJUMP")
public class DecodeAutonomousV3 extends OpMode {

    // ── State machines ───────────────────────────────────────────────────────
    public enum InitState {
        HARDWARE, SELECT_ALLIANCE, COMPUTE_POSES, BUILD_PATHS, READY
    }

    public enum PathState {
        INITIAL,
        // 1. Shoot preload
        MOVE_PRELOAD,       SHOOT_PRELOAD,
        // 2-3. B2
        GRAB_B2,            WAIT_B2,        RETURN_B2,      SHOOT_B2,
        // 4-5. Gate (first trip)
        GRAB_GATE_1,        HOLD_GATE_1,    RETURN_GATE_1,  SHOOT_GATE_1,
        // 6-7. Gate (second trip)
        GRAB_GATE_2,        HOLD_GATE_2,    RETURN_GATE_2,  SHOOT_GATE_2,
        // 8-9. B1
        GRAB_B1,            WAIT_B1,        RETURN_B1,      SHOOT_B1,
        // 10-11. B3
        GRAB_B3,            WAIT_B3,        RETURN_B3,      SHOOT_B3,
        IDLE_PARKED
    }

    // ── Timing constants ─────────────────────────────────────────────────────
    private static final double SHOOT_DURATION     = 2.0;   // feed time per shoot window
    private static final double VELOCITY_THRESHOLD = 240.0; // ticks/s — shooter ready threshold
    private static final double VELOCITY_WAIT_MAX  = 0.5;   // max wait for shooter to spin up
    private static final double GRAB_DWELL         = 0.25;  // pause at B1/B2/B3 after arriving
    private static final double GATE_HOLD_TIME     = 2.0;   // hold lever down so balls roll in
    private static final double MATCH_DURATION     = 20.0;  // hard kill at 20s

    // ── Hardware & state ─────────────────────────────────────────────────────
    private InitState  initState  = InitState.HARDWARE;
    private PathState  pathState  = PathState.INITIAL;
    private double     alliance;       // 1 = BLUE, -1 = RED
    private String     allianceName;
    private Follower   follower;
    private Timer      pathTimer;
    private ElapsedTime matchTimer;

    private final double SHOOTER_VELOCITY = 570;

    // ── Poses ────────────────────────────────────────────────────────────────
    private Pose startPose, shootPose;
    // B1 — circle (y≈84)
    private Pose cp1B1, cp2B1, endB1;
    // B2 — upper oval (y≈60)
    private Pose cp1B2, cp2B2, endB2;
    // Gate — lever at x=56, y=36, heading=41°
    private Pose cp1Gate, endGate;
    // B3 — lower oval (y≈36)
    private Pose cp1B3, cp2B3, endB3;

    // ── Paths ────────────────────────────────────────────────────────────────
    private PathChain toShootPose;
    private PathChain toB2,     fromB2;
    private PathChain toGate,   fromGate;   // reused for both gate trips
    private PathChain toB1,     fromB1;
    private PathChain toB3,     fromB3;

    // ── Motors ───────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    // ════════════════════════════════════════════════════════════════════════
    public void init() {}

    // ════════════════════════════════════════════════════════════════════════
    @Override
    public void init_loop() {
        switch (initState) {

            case HARDWARE:
                telemetry.addLine("Initializing hardware...");
                leftShooter   = hardwareMap.get(DcMotorEx.class, "LS");
                rightShooter  = hardwareMap.get(DcMotorEx.class, "RS");
                intakeMotor   = hardwareMap.get(DcMotorEx.class, "IN");
                transferMotor = hardwareMap.get(DcMotorEx.class, "TR");
                shooters      = new DcMotorEx[]{leftShooter, rightShooter};

                PIDFCoefficients coeffs = new PIDFCoefficients(45.0, 0.02, 2.5, 13.2);
                for (DcMotorEx m : shooters) {
                    m.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
                    m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                }
                leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
                rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
                transferMotor.setDirection(DcMotorEx.Direction.FORWARD);
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

                follower  = Constants.createFollower(hardwareMap);
                pathTimer = new Timer();
                initState = InitState.SELECT_ALLIANCE;
                break;

            case SELECT_ALLIANCE:
                telemetry.addLine("Select Alliance:");
                telemetry.addLine("  LEFT  bumper = BLUE");
                telemetry.addLine("  RIGHT bumper = RED");
                if (gamepad1.left_bumper) {
                    allianceName = "BLUE";
                    DataPasser.currentAlliance = DataPasser.Alliance.BLUE;
                    initState = InitState.COMPUTE_POSES;
                }
                if (gamepad1.right_bumper) {
                    allianceName = "RED";
                    DataPasser.currentAlliance = DataPasser.Alliance.RED;
                    initState = InitState.COMPUTE_POSES;
                }
                alliance = "BLUE".equals(allianceName) ? 1 : -1;
                break;

            case COMPUTE_POSES:
                telemetry.addLine("Computing positions...");

                startPose = new Pose(72 - (72 - 15.78) * alliance, 113.52,
                        Math.toRadians(90 + 90 * alliance));
                shootPose = new Pose(72 - (72 - 48) * alliance, 96,
                        Math.toRadians(90 + 45 * alliance));

                // B1 — circle (y≈84)
                cp1B1 = new Pose(72 - (72 - 48) * alliance, 84);
                cp2B1 = new Pose(72 - (72 - 36) * alliance, 84);
                endB1 = new Pose(72 - (72 - 18) * alliance, 84,
                        Math.toRadians(90 - 90 * alliance));

                // B2 — upper oval (y≈60)
                cp1B2 = new Pose(72 - (72 - 48) * alliance, 60);
                cp2B2 = new Pose(72 - (72 - 36) * alliance, 60);
                endB2 = new Pose(72 - (72 - 12) * alliance, 60,
                        Math.toRadians(90 - 90 * alliance));

                // Gate — lever position from field map screenshot
                // x=56, y=36, heading=41° (mirrored for RED)
                cp1Gate = new Pose(72 - (72 - 48) * alliance, 60);
                endGate = new Pose(72 - (72 - 56) * alliance, 36,
                        Math.toRadians(41 * alliance));

                // B3 — lower oval (y≈36)
                cp1B3 = new Pose(72 - (72 - 48) * alliance, 36);
                cp2B3 = new Pose(72 - (72 - 36) * alliance, 36);
                endB3 = new Pose(72 - (72 - 12) * alliance, 36,
                        Math.toRadians(90 - 90 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                telemetry.addLine("Building paths...");

                // start → shootPose
                toShootPose = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(),
                                shootPose.getHeading(), 0.75)
                        .build();

                // shoot ↔ B2
                toB2 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, cp1B2, cp1B2, cp2B2, endB2))
                        .setConstantHeadingInterpolation(endB2.getHeading())
                        .build();
                fromB2 = follower.pathBuilder()
                        .addPath(new BezierLine(endB2, shootPose))
                        .setLinearHeadingInterpolation(endB2.getHeading(),
                                shootPose.getHeading(), 0.75)
                        .build();

                // shoot ↔ Gate (same path reused for both gate trips)
                toGate = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, cp1Gate, endGate))
                        .setLinearHeadingInterpolation(shootPose.getHeading(),
                                endGate.getHeading(), 0.75)
                        .build();
                fromGate = follower.pathBuilder()
                        .addPath(new BezierLine(endGate, shootPose))
                        .setLinearHeadingInterpolation(endGate.getHeading(),
                                shootPose.getHeading(), 0.75)
                        .build();

                // shoot ↔ B1
                toB1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, cp1B1, cp1B1, cp2B1, endB1))
                        .setConstantHeadingInterpolation(endB1.getHeading())
                        .build();
                fromB1 = follower.pathBuilder()
                        .addPath(new BezierLine(endB1, shootPose))
                        .setLinearHeadingInterpolation(endB1.getHeading(),
                                shootPose.getHeading(), 0.75)
                        .build();

                // shoot ↔ B3
                toB3 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, cp1B3, cp1B3, cp2B3, endB3))
                        .setConstantHeadingInterpolation(endB3.getHeading())
                        .build();
                fromB3 = follower.pathBuilder()
                        .addPath(new BezierLine(endB3, shootPose))
                        .setLinearHeadingInterpolation(endB3.getHeading(),
                                shootPose.getHeading(), 0.75)
                        .build();

                follower.setStartingPose(startPose);
                initState = InitState.READY;
                break;

            case READY:
                telemetry.addLine("READY! Waiting for start...");
                break;
        }

        telemetry.addData("Alliance", allianceName != null ? allianceName : "Not Selected");
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        // Hard 20s kill switch
        if (matchTimer != null && matchTimer.seconds() >= MATCH_DURATION) {
            stopAllMotors();
            pathState = PathState.IDLE_PARKED;
        }

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Match Time",    "%.2f / %.0fs", matchTimer != null ? matchTimer.seconds() : 0.0, MATCH_DURATION);
        telemetry.addData("State Time",    "%.2fs", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Shooter L",     "%.0f t/s", leftShooter.getVelocity());
        telemetry.addData("Shooter R",     "%.0f t/s", rightShooter.getVelocity());
        telemetry.addData("Vel Ready",     shooterReady() ? "YES" : "NO");
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════════════
    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── INITIAL ──────────────────────────────────────────────────────
            case INITIAL:
                matchTimer = new ElapsedTime();
                for (DcMotorEx m : shooters) m.setVelocity(SHOOTER_VELOCITY);
                intakeMotor.setPower(1.0);
                follower.followPath(toShootPose, true);
                setPathState(PathState.MOVE_PRELOAD);
                break;

            case MOVE_PRELOAD:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_PRELOAD);
                break;

            // ── 1. SHOOT PRELOAD ─────────────────────────────────────────────
            case SHOOT_PRELOAD:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_DURATION) {
                    transferMotor.setPower(-0.6);
                    follower.followPath(toB2, 0.75, true);
                    setPathState(PathState.GRAB_B2);
                }
                break;

            // ── 2-3. B2 ──────────────────────────────────────────────────────
            case GRAB_B2:
                if (!follower.isBusy()) setPathState(PathState.WAIT_B2);
                break;

            case WAIT_B2:
                if (pathTimer.getElapsedTimeSeconds() >= GRAB_DWELL) {
                    follower.followPath(fromB2, true);
                    setPathState(PathState.RETURN_B2);
                }
                break;

            case RETURN_B2:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_B2);
                break;

            case SHOOT_B2:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_DURATION) {
                    transferMotor.setPower(-0.6);
                    follower.followPath(toGate, true);
                    setPathState(PathState.GRAB_GATE_1);
                }
                break;

            // ── 4-5. GATE (first trip) ────────────────────────────────────────
            // Robot drives to lever pose and holds it down — balls roll into intake
            case GRAB_GATE_1:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) setPathState(PathState.HOLD_GATE_1);
                break;

            case HOLD_GATE_1:
                // Stay pressed on lever — intake running — balls roll down ramp
                intakeMotor.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() >= GATE_HOLD_TIME) {
                    follower.followPath(fromGate, true);
                    setPathState(PathState.RETURN_GATE_1);
                }
                break;

            case RETURN_GATE_1:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_GATE_1);
                break;

            case SHOOT_GATE_1:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_DURATION) {
                    transferMotor.setPower(-0.6);
                    follower.followPath(toGate, true);
                    setPathState(PathState.GRAB_GATE_2);
                }
                break;

            // ── 6-7. GATE (second trip) ───────────────────────────────────────
            case GRAB_GATE_2:
                intakeMotor.setPower(1.0);
                if (!follower.isBusy()) setPathState(PathState.HOLD_GATE_2);
                break;

            case HOLD_GATE_2:
                intakeMotor.setPower(1.0);
                if (pathTimer.getElapsedTimeSeconds() >= GATE_HOLD_TIME) {
                    follower.followPath(fromGate, true);
                    setPathState(PathState.RETURN_GATE_2);
                }
                break;

            case RETURN_GATE_2:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_GATE_2);
                break;

            case SHOOT_GATE_2:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_DURATION) {
                    transferMotor.setPower(-0.6);
                    follower.followPath(toB1, 0.75, true);
                    setPathState(PathState.GRAB_B1);
                }
                break;

            // ── 8-9. B1 ──────────────────────────────────────────────────────
            case GRAB_B1:
                if (!follower.isBusy()) setPathState(PathState.WAIT_B1);
                break;

            case WAIT_B1:
                if (pathTimer.getElapsedTimeSeconds() >= GRAB_DWELL) {
                    follower.followPath(fromB1, true);
                    setPathState(PathState.RETURN_B1);
                }
                break;

            case RETURN_B1:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_B1);
                break;

            case SHOOT_B1:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                if (pathTimer.getElapsedTimeSeconds() >= SHOOT_DURATION) {
                    transferMotor.setPower(-0.6);
                    follower.followPath(toB3, 0.75, true);
                    setPathState(PathState.GRAB_B3);
                }
                break;

            // ── 10-11. B3 ────────────────────────────────────────────────────
            case GRAB_B3:
                if (!follower.isBusy()) setPathState(PathState.WAIT_B3);
                break;

            case WAIT_B3:
                if (pathTimer.getElapsedTimeSeconds() >= GRAB_DWELL) {
                    follower.followPath(fromB3, true);
                    setPathState(PathState.RETURN_B3);
                }
                break;

            case RETURN_B3:
                if (!follower.isBusy()) setPathState(PathState.SHOOT_B3);
                break;

            // Final shoot — runs until the 20s hard cap fires
            case SHOOT_B3:
                if (shooterReady() || pathTimer.getElapsedTimeSeconds() >= VELOCITY_WAIT_MAX) {
                    transferMotor.setPower(0.4);
                }
                break;

            case IDLE_PARKED:
                break;
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    private boolean shooterReady() {
        return Math.abs(leftShooter.getVelocity())  >= VELOCITY_THRESHOLD
                && Math.abs(rightShooter.getVelocity()) >= VELOCITY_THRESHOLD;
    }

    private void stopAllMotors() {
        for (DcMotorEx m : shooters) m.setPower(0);
        transferMotor.setPower(0);
        intakeMotor.setPower(0);
        follower.breakFollowing();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}