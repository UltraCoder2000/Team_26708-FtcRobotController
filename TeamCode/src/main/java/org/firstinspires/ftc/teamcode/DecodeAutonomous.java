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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DECODE Autonomous (Near)", preselectTeleOp = "DECODE TeleOp")
public class DecodeAutonomous extends OpMode {
    public enum InitState {
        HARDWARE,
        SELECT_ALLIANCE,
        COMPUTE_POSES,
        BUILD_PATHS,
        READY
    }
    public enum PathState {
        INITIAL,
        MOVE_PRELOAD,
        SHOOT_PRELOAD,
        GRAB_PICKUP_1,
        WAIT_PICKUP_1,
        MOVE_OPEN_GATE,
        MOVE_PICKUP_1,
        SHOOT_PICKUP_1,
        GRAB_PICKUP_2,
        WAIT_PICKUP_2,
        MOVE_PICKUP_2,
        SHOOT_PICKUP_2,
        GRAB_PICKUP_3,
        WAIT_PICKUP_3,
        MOVE_PICKUP_3,
        SHOOT_PICKUP_3,
        IDLE_PARKED
    }
    private InitState initState = InitState.HARDWARE;
    private PathState pathState = PathState.INITIAL;
    private double alliance;
    private String allianceName;
    private Follower follower;
    private Timer pathTimer;
    private final double SHOOTER_VELOCITY = 570;
    private Pose
            startPose,
            shootPose,
            firstControlPosePickup1,
            secondControlPosePickup1,
            endPosePickup1,
            firstControlPoseGate,
            secondControlPoseGate,
            endPoseGate,
            firstControlPosePickup2,
            secondControlPosePickup2,
            endPosePickup2,
            firstControlPosePickup3,
            secondControlPosePickup3,
            endPosePickup3,
            finalPose;
    private PathChain
            scorePreload,
            grabPickup1,
            openGate,
            scorePickup1,
            grabPickup2,
            scorePickup2,
            grabPickup3,
            scorePickup3;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    public void init() {}

    @Override
    public void init_loop() {
        switch (initState) {
            case HARDWARE:
                telemetry.addLine("Initializing hardware...");

                leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
                rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
                intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
                transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

                shooters = new DcMotorEx[]{leftShooter, rightShooter};

                PIDFCoefficients coeffs = new PIDFCoefficients(45.0, 0.02, 2.5, 13.2);
                for (DcMotorEx motor : shooters) {
                    motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
                    motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                }

                leftShooter.setDirection(DcMotorEx.Direction.REVERSE);
                rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
                transferMotor.setDirection(DcMotorEx.Direction.FORWARD);
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

                follower = Constants.createFollower(hardwareMap);
                pathTimer = new Timer();

                initState = InitState.SELECT_ALLIANCE;
                break;

            case SELECT_ALLIANCE:
                telemetry.addLine("Select Alliance (GAMEPAD 1):");
                telemetry.addLine("Left bumper = BLUE");
                telemetry.addLine("Right bumper = RED");

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
                startPose = new Pose(72 - (72 - 15.78) * alliance, 113.52, Math.toRadians(90 + 90 * alliance));
                shootPose = new Pose(72 - (72 - 48) * alliance, 96, Math.toRadians(90 + 45 * alliance));
                firstControlPosePickup1 = new Pose(72 - (72 - 48) * alliance, 84);
                secondControlPosePickup1 = new Pose(72 - (72 - 36) * alliance, 84);
                endPosePickup1 = new Pose(72 - (72 - 18) * alliance, 84, Math.toRadians(90 - 90 * alliance));
                firstControlPoseGate = new Pose(72 - (72 - 48) * alliance, 84);
                secondControlPoseGate = new Pose(72 - (72 - 48) * alliance, 72);
                endPoseGate = new Pose(72 - (72 - 16.38) * alliance, 72, Math.toRadians(90 + 90.1 * alliance));
                firstControlPosePickup2 = new Pose(72 - (72 - 48) * alliance, 60);
                secondControlPosePickup2 = new Pose(72 - (72 - 36) * alliance, 60);
                endPosePickup2 = new Pose(72 - (72 - 12) * alliance, 60, Math.toRadians(90 - 90 * alliance));
                firstControlPosePickup3 = new Pose(72 - (72 - 48) * alliance, 36);
                secondControlPosePickup3 = new Pose(72 - (72 - 36) * alliance, 36);
                endPosePickup3 = new Pose(72 - (72 - 12) * alliance, 36, Math.toRadians(90 - 90 * alliance));
                finalPose = new Pose(72 - (72 - 58.79) * alliance, 110.06, Math.toRadians(90 + 60 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                telemetry.addLine("Building Paths...");
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, firstControlPosePickup1, firstControlPosePickup1, secondControlPosePickup1, endPosePickup1))
                        .setConstantHeadingInterpolation(endPosePickup1.getHeading())
                        .build();

                openGate = follower.pathBuilder()
                        .addPath(new BezierCurve(endPosePickup1, firstControlPoseGate, secondControlPoseGate, endPoseGate))
                        .setLinearHeadingInterpolation(endPosePickup1.getHeading(), endPoseGate.getHeading(), 0.5)
                        .build();

                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(endPoseGate, shootPose))
                        .setLinearHeadingInterpolation(endPoseGate.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup2 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, firstControlPosePickup2, firstControlPosePickup2, secondControlPosePickup2, endPosePickup2))
                        .setConstantHeadingInterpolation(endPosePickup2.getHeading())
                        .build();

                scorePickup2 = follower.pathBuilder()
                        .addPath(new BezierLine(endPosePickup2, shootPose))
                        .setLinearHeadingInterpolation(endPosePickup2.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup3 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, firstControlPosePickup3, firstControlPosePickup3, secondControlPosePickup3, endPosePickup3))
                        .setConstantHeadingInterpolation(endPosePickup3.getHeading())
                        .build();

                scorePickup3 = follower.pathBuilder()
                        .addPath(new BezierLine(endPosePickup3, finalPose))
                        .setLinearHeadingInterpolation(endPosePickup3.getHeading(), finalPose.getHeading(), 0.75)
                        .build();

                follower.setStartingPose(startPose);
                initState = InitState.READY;
                break;

            case READY:
                telemetry.addLine("READY!");
                break;
        }

        if (allianceName != null) {
            telemetry.addData("Alliance", allianceName);
        } else {
            telemetry.addData("Alliance", "Not Selected");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case INITIAL:
                follower.followPath(scorePreload, true);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(SHOOTER_VELOCITY);
                }
                setPathState(PathState.MOVE_PRELOAD);
                break;

            case MOVE_PRELOAD:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                intakeMotor.setPower(1);
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                transferMotor.setPower(-0.6);
                follower.followPath(grabPickup1, 0.6, true);
                setPathState(PathState.GRAB_PICKUP_1);
                break;

            case GRAB_PICKUP_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_1);
                }
                break;

            case WAIT_PICKUP_1:
                if (pathTimer.getElapsedTimeSeconds() < 0.25) {
                    break;
                }
                follower.followPath(openGate, true);
                setPathState(PathState.MOVE_OPEN_GATE);
                break;

            case MOVE_OPEN_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(PathState.MOVE_PICKUP_1);
                }
                break;

            case MOVE_PICKUP_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_1);
                }
                break;

            case SHOOT_PICKUP_1:
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                transferMotor.setPower(-0.6);
                follower.followPath(grabPickup2, 0.6, true);
                setPathState(PathState.GRAB_PICKUP_2);
                break;

            case GRAB_PICKUP_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_2);
                }
                break;

            case WAIT_PICKUP_2:
                if (pathTimer.getElapsedTimeSeconds() < 0.25) {
                    break;
                }
                follower.followPath(scorePickup2, true);
                setPathState(PathState.MOVE_PICKUP_2);
                break;

            case MOVE_PICKUP_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_2);
                }
                break;

            case SHOOT_PICKUP_2:
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                transferMotor.setPower(-0.6);
                follower.followPath(grabPickup3, 0.6, true);
                setPathState(PathState.GRAB_PICKUP_3);
                break;

            case GRAB_PICKUP_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_3);
                }
                break;

            case WAIT_PICKUP_3:
                if (pathTimer.getElapsedTimeSeconds() < 0.25) {
                    break;
                }
                follower.followPath(scorePickup3, true);
                setPathState(PathState.MOVE_PICKUP_3);
                break;

            case MOVE_PICKUP_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_3);
                }
                break;

            case SHOOT_PICKUP_3:
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                leftShooter.setPower(0);
                rightShooter.setPower(0);
                transferMotor.setPower(0);
                intakeMotor.setPower(0);
                setPathState(PathState.IDLE_PARKED);
                break;

            case IDLE_PARKED:
                break;
        }
    }

    public void setPathState(PathState pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}