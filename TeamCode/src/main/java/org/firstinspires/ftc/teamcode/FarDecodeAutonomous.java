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

@Autonomous(name = "DECODE Autonomous (Far)", preselectTeleOp = "DECODE TeleOp")
public class FarDecodeAutonomous extends OpMode {
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
        GRAB_SPIKE_MARK,
        WAIT_SPIKE_MARK,
        MOVE_SPIKE_MARK,
        SHOOT_SPIKE_MARK,
        MOVE_OPTIONAL_PARK,
        WAIT_OPTIONAL_PARK,
        START_GRAB_LOADING_ZONE,
        END_GRAB_LOADING_ZONE,
        WAIT_LOADING_ZONE,
        MOVE_LOADING_ZONE,
        SHOOT_LOADING_ZONE,
        MOVE_FINAL_PARK,
        IDLE_PARKED
    }
    private InitState initState = InitState.HARDWARE;
    private PathState pathState = PathState.INITIAL;
    private double alliance;
    private String allianceName;
    private Follower follower;
    private Timer pathTimer;
    private final double SHOOTER_VELOCITY = 750;
    private Pose
            startPose,
            shootPose,
            optionalParkPose,
            spikeMarkControlPose1,
            spikeMarkControlPose2,
            spikeMarkControlPose3,
            spikeMarkEndPose,
            loadingZoneStartPose,
            loadingZoneEndPose,
            finalPose;
    private PathChain
            scorePreload,
            startLoadingZonePickup,
            endLoadingZonePickup,
            scoreLoadingZonePickup,
            scoreOptionalPark,
            grabSpikeMarkPickup,
            scoreSpikeMarkPickup,
            scoreFinalPark;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    @Override
    public void init() {
    }

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
                startPose = new Pose(72 - (72 - 60.000) * alliance, 7.950, Math.toRadians(90 + 90 * alliance));
                shootPose = new Pose(72 - (72 - 60.000) * alliance, 21.000, Math.toRadians(90 + 26 * alliance));
                loadingZoneStartPose = new Pose(72 - (72 - 11.000) * alliance, 21.000, Math.toRadians(90 - 70 * alliance));
                loadingZoneEndPose = new Pose(72 - (72 - 11.000) * alliance, 11.000, Math.toRadians(90 - 70 * alliance));
                optionalParkPose = new Pose(72 - (72 - 36.000) * alliance, 9.000, Math.toRadians(90 - 90 * alliance));
                spikeMarkControlPose1 = new Pose(72 - (72 - 60.000) * alliance, 24.000);
                spikeMarkControlPose2 = new Pose(72 - (72 - 60.000) * alliance, 36.000);
                spikeMarkControlPose3 = new Pose(72 - (72 - 36.000) * alliance, 36.000);
                spikeMarkEndPose = new Pose(72 - (72 - 9.000) * alliance, 36.000, Math.toRadians(90 - 90 * alliance));
                finalPose = new Pose(72 - (72 - 54.000) * alliance, 33.300, Math.toRadians(90 + 26 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                telemetry.addLine("Building Paths...");
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                        .build();

                startLoadingZonePickup = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, loadingZoneStartPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), loadingZoneStartPose.getHeading())
                        .build();

                endLoadingZonePickup = follower.pathBuilder()
                        .addPath(new BezierLine(loadingZoneStartPose, loadingZoneEndPose))
                        .setConstantHeadingInterpolation(loadingZoneEndPose.getHeading())
                        .build();

                scoreLoadingZonePickup = follower.pathBuilder()
                        .addPath(new BezierLine(loadingZoneEndPose, shootPose))
                        .setLinearHeadingInterpolation(loadingZoneEndPose.getHeading(), shootPose.getHeading())
                        .build();

                scoreOptionalPark = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, optionalParkPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), optionalParkPose.getHeading())
                        .build();

                grabSpikeMarkPickup = follower.pathBuilder()
                        .addPath(new BezierCurve(optionalParkPose, spikeMarkControlPose1, spikeMarkControlPose2, spikeMarkControlPose3, spikeMarkEndPose))
                        .setConstantHeadingInterpolation(spikeMarkEndPose.getHeading())
                        .build();

                scoreSpikeMarkPickup = follower.pathBuilder()
                        .addPath(new BezierLine(spikeMarkEndPose, shootPose))
                        .setLinearHeadingInterpolation(spikeMarkEndPose.getHeading(), shootPose.getHeading())
                        .build();

                scoreFinalPark = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, finalPose))
                        .setConstantHeadingInterpolation(finalPose.getHeading())
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
                transferMotor.setPower(-0.4);
                follower.followPath(startLoadingZonePickup, 0.6, true);
                setPathState(PathState.START_GRAB_LOADING_ZONE);
                break;

            case START_GRAB_LOADING_ZONE:
                if (!follower.isBusy()) {
                    follower.followPath(endLoadingZonePickup, 0.4, true);
                    setPathState(PathState.END_GRAB_LOADING_ZONE);
                }
                break;

            case END_GRAB_LOADING_ZONE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_LOADING_ZONE);
                }
                break;

            case WAIT_LOADING_ZONE:
                if (pathTimer.getElapsedTimeSeconds() < 0.5) {
                    break;
                }
                follower.followPath(scoreLoadingZonePickup, true);
                setPathState(PathState.MOVE_LOADING_ZONE);
                break;

            case MOVE_LOADING_ZONE:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_LOADING_ZONE);
                }
                break;

            case SHOOT_LOADING_ZONE:
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                transferMotor.setPower(-0.4);
                follower.followPath(scoreOptionalPark, true);
                setPathState(PathState.MOVE_OPTIONAL_PARK);
                break;

            case MOVE_OPTIONAL_PARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_OPTIONAL_PARK);
                }
                break;

            case WAIT_OPTIONAL_PARK:
                if (pathTimer.getElapsedTimeSeconds() < 3) {
                    break;
                }
                follower.followPath(grabSpikeMarkPickup, 0.6, true);
                setPathState(PathState.GRAB_SPIKE_MARK);
                break;

            case GRAB_SPIKE_MARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_SPIKE_MARK);
                }
                break;

            case WAIT_SPIKE_MARK:
                if (pathTimer.getElapsedTimeSeconds() < 0.5) {
                    break;
                }
                follower.followPath(scoreSpikeMarkPickup, true);
                setPathState(PathState.MOVE_SPIKE_MARK);
                break;

            case MOVE_SPIKE_MARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SPIKE_MARK);
                }
                break;

            case SHOOT_SPIKE_MARK:
                transferMotor.setPower(0.4);
                if (pathTimer.getElapsedTimeSeconds() < 1.5) {
                    break;
                }
                leftShooter.setPower(0);
                rightShooter.setPower(0);
                transferMotor.setPower(0);
                intakeMotor.setPower(0);
                follower.followPath(scoreFinalPark, true);
                setPathState(PathState.MOVE_FINAL_PARK);
                break;

            case MOVE_FINAL_PARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE_PARKED);
                }
                break;

            case IDLE_PARKED:
                break;
        }
    }

    public void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}