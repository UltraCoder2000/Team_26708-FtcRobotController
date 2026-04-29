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
        GRAB_PICKUP_MIDDLE_SPIKE,
        WAIT_PICKUP_MIDDLE_SPIKE,
        SCORE_PICKUP_MIDDLE_SPIKE,
        SHOOT_PICKUP_MIDDLE_SPIKE,
        GRAB_PICKUP_GATE,
        INTAKE_PICKUP_GATE,
        WAIT_PICKUP_GATE,
        SCORE_PICKUP_GATE,
        SHOOT_PICKUP_GATE,
        GRAB_PICKUP_CLOSE_SPIKE,
        WAIT_PICKUP_CLOSE_SPIKE,
        SCORE_PICKUP_CLOSE_SPIKE,
        SHOOT_PICKUP_CLOSE_SPIKE,
        GRAB_PICKUP_FAR_SPIKE,
        WAIT_PICKUP_FAR_SPIKE,
        SCORE_PICKUP_FAR_SPIKE,
        SHOOT_PICKUP_FAR_SPIKE,
        PARKING
    }
    private InitState initState = InitState.HARDWARE;
    private PathState pathState = PathState.INITIAL;
    private double alliance;
    private String allianceName;
    private Follower follower;
    private Timer pathTimer;
    private final double SHOOTER_VELOCITY = 610; // Ticks per second
    private final double BACKWARDS_VELOCITY = -450; // Ticks per second
    private Pose
            startPose,
            shootPose,
            pickupMiddleSpikeControlPose,
            pickupMiddleSpikeConnectorPose,
            pickupMiddleSpikeEndPose,
            scoreMiddleSpikeControlPose,
            pickupGateControlPose,
            pickupGateConnectorPose,
            pickupGateEndPose,
            pickupGateIntakePose,
            scoreGateControlPose,
            pickupCloseSpikeConnectorPose,
            pickupCloseSpikeEndPose,
            pickupFarSpikeControlPose,
            pickupFarSpikeConnectorPose,
            pickupFarSpikeEndPose,
            finalPose;
    private PathChain
            scorePreload,
            grabPickupMiddleSpike,
            scorePickupMiddleSpike,
            grabPickupGate,
            intakePickupGate,
            scorePickupGate,
            grabPickupCloseSpike,
            scorePickupCloseSpike,
            grabPickupFarSpike,
            scorePickupFarSpike,
            finalPark;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    public void init() {}

    public void stop() {
        DataPasser.endAutoPose = follower.getPose();
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

                PIDFCoefficients coeffs = new PIDFCoefficients(90.0, 0.02, 2.5, 13.2);
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
                shootPose = new Pose(72 - (72 - 60) * alliance, 84, Math.toRadians(90 + 45 * alliance));
                pickupMiddleSpikeControlPose = new Pose(72 - (72 - 60) * alliance, 60);
                pickupMiddleSpikeConnectorPose = new Pose(72 - (72 - 40) * alliance, 60, Math.toRadians(90 - 90 * alliance));
                pickupMiddleSpikeEndPose = new Pose(72 - (72 - 11) * alliance, 60, Math.toRadians(90 - 90 * alliance));
                scoreMiddleSpikeControlPose = new Pose(72 - (72 - 36) * alliance, 60);
                pickupGateControlPose = new Pose(72 - (72 - 60) * alliance, 63);
                pickupGateConnectorPose = new Pose(72 - (72 - 40) * alliance, 63, Math.toRadians(90 - 105 * alliance));
                pickupGateEndPose = new Pose(72 - (72 - 13) * alliance, 63, Math.toRadians(90 - 105 * alliance));
                pickupGateIntakePose = new Pose(72 - (72 - 10) * alliance, 58, Math.toRadians(90 - 115 * alliance));
                scoreGateControlPose = new Pose(72 - (72 - 38) * alliance, 58);
                pickupCloseSpikeConnectorPose = new Pose(72 - (72 - 40) * alliance, 84, Math.toRadians(90 - 90 * alliance));
                pickupCloseSpikeEndPose = new Pose(72 - (72 - 19) * alliance, 84, Math.toRadians(90 - 90 * alliance));
                pickupFarSpikeControlPose = new Pose(72 - (72 - 60) * alliance, 37);
                pickupFarSpikeConnectorPose = new Pose(72 - (72 - 40) * alliance, 37, Math.toRadians(90 - 90 * alliance));
                pickupFarSpikeEndPose = new Pose(72 - (72 - 11) * alliance, 37, Math.toRadians(90 - 90 * alliance));
                finalPose = new Pose(72 - (72 - 60) * alliance, 110, Math.toRadians(90 + 45 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                telemetry.addLine("Building Paths...");

                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickupMiddleSpike = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, pickupMiddleSpikeControlPose, pickupMiddleSpikeConnectorPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), pickupMiddleSpikeConnectorPose.getHeading(), 0.75)
                        .addPath(new BezierLine(pickupMiddleSpikeConnectorPose, pickupMiddleSpikeEndPose))
                        .setConstantHeadingInterpolation(pickupMiddleSpikeEndPose.getHeading())
                        .build();

                scorePickupMiddleSpike = follower.pathBuilder()
                        .addPath(new BezierCurve(pickupMiddleSpikeEndPose, scoreMiddleSpikeControlPose, shootPose))
                        .setLinearHeadingInterpolation(pickupMiddleSpikeEndPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickupGate = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, pickupGateControlPose, pickupGateConnectorPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), pickupGateConnectorPose.getHeading(), 0.75)
                        .addPath(new BezierLine(pickupGateConnectorPose, pickupGateEndPose))
                        .setConstantHeadingInterpolation(pickupGateEndPose.getHeading())
                        .build();

                intakePickupGate = follower.pathBuilder()
                        .addPath(new BezierLine(pickupGateEndPose, pickupGateIntakePose))
                        .setLinearHeadingInterpolation(pickupGateEndPose.getHeading(), pickupGateIntakePose.getHeading())
                        .build();

                scorePickupGate = follower.pathBuilder()
                        .addPath(new BezierCurve(pickupGateIntakePose, scoreGateControlPose, shootPose))
                        .setLinearHeadingInterpolation(pickupGateEndPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickupCloseSpike = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, pickupCloseSpikeConnectorPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), pickupCloseSpikeConnectorPose.getHeading(), 0.75)
                        .addPath(new BezierLine(pickupCloseSpikeConnectorPose, pickupCloseSpikeEndPose))
                        .setConstantHeadingInterpolation(pickupCloseSpikeEndPose.getHeading())
                        .build();

                scorePickupCloseSpike = follower.pathBuilder()
                        .addPath(new BezierLine(pickupCloseSpikeEndPose, shootPose))
                        .setLinearHeadingInterpolation(pickupCloseSpikeEndPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickupFarSpike = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, pickupFarSpikeControlPose, pickupFarSpikeConnectorPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), pickupFarSpikeConnectorPose.getHeading(), 0.75)
                        .addPath(new BezierLine(pickupFarSpikeConnectorPose, pickupFarSpikeEndPose))
                        .setConstantHeadingInterpolation(pickupFarSpikeEndPose.getHeading())
                        .build();

                scorePickupFarSpike = follower.pathBuilder()
                        .addPath(new BezierLine(pickupFarSpikeEndPose, shootPose))
                        .setLinearHeadingInterpolation(pickupFarSpikeEndPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                finalPark = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, finalPose))
                        .setConstantHeadingInterpolation(shootPose.getHeading())
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
                transferMotor.setPower(-0.4);
                setPathState(PathState.MOVE_PRELOAD);
                break;

            case MOVE_PRELOAD:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(BACKWARDS_VELOCITY);
                    }
                    transferMotor.setPower(0.2);
                    follower.followPath(grabPickupMiddleSpike, true);
                    setPathState(PathState.GRAB_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case GRAB_PICKUP_MIDDLE_SPIKE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case WAIT_PICKUP_MIDDLE_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(-0.4);
                    follower.followPath(scorePickupMiddleSpike, true);
                    setPathState(PathState.SCORE_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case SCORE_PICKUP_MIDDLE_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(SHOOTER_VELOCITY);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_MIDDLE_SPIKE);
                }
                break;

            case SHOOT_PICKUP_MIDDLE_SPIKE:
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(BACKWARDS_VELOCITY);
                    }
                    transferMotor.setPower(0.2);
                    follower.followPath(grabPickupGate, true);
                    setPathState(PathState.GRAB_PICKUP_GATE);
                }
                break;

            case GRAB_PICKUP_GATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_PICKUP_GATE);
                    follower.followPath(intakePickupGate);
                }
                break;

            case INTAKE_PICKUP_GATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_GATE);
                }
                break;

            case WAIT_PICKUP_GATE:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(-0.4);
                    follower.followPath(scorePickupGate, true);
                    setPathState(PathState.SCORE_PICKUP_GATE);
                }
                break;

            case SCORE_PICKUP_GATE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(SHOOTER_VELOCITY);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_GATE);
                }
                break;

            case SHOOT_PICKUP_GATE:
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(BACKWARDS_VELOCITY);
                    }
                    transferMotor.setPower(0.2);
                    follower.followPath(grabPickupCloseSpike, true);
                    setPathState(PathState.GRAB_PICKUP_CLOSE_SPIKE);
                }
                break;

            case GRAB_PICKUP_CLOSE_SPIKE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_CLOSE_SPIKE);
                }
                break;

            case WAIT_PICKUP_CLOSE_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(-0.4);
                    follower.followPath(scorePickupCloseSpike, true);
                    setPathState(PathState.SCORE_PICKUP_CLOSE_SPIKE);
                }
                break;

            case SCORE_PICKUP_CLOSE_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(SHOOTER_VELOCITY);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_CLOSE_SPIKE);
                }
                break;

            case SHOOT_PICKUP_CLOSE_SPIKE:
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(BACKWARDS_VELOCITY);
                    }
                    transferMotor.setPower(0.2);
                    follower.followPath(grabPickupFarSpike, true);
                    setPathState(PathState.GRAB_PICKUP_FAR_SPIKE);
                }
                break;

            case GRAB_PICKUP_FAR_SPIKE:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_PICKUP_FAR_SPIKE);
                }
                break;

            case WAIT_PICKUP_FAR_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.75) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(-0.4);
                    follower.followPath(scorePickupFarSpike, true);
                    setPathState(PathState.SCORE_PICKUP_FAR_SPIKE);
                }
                break;

            case SCORE_PICKUP_FAR_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    for (DcMotorEx motor : shooters) {
                        motor.setVelocity(SHOOTER_VELOCITY);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_FAR_SPIKE);
                }
                break;

            case SHOOT_PICKUP_FAR_SPIKE:
                intakeMotor.setPower(1);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    follower.followPath(finalPark, true);
                    setPathState(PathState.PARKING);
                }
                break;

            case PARKING:
                break;
        }
    }

    public void setPathState(PathState pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}