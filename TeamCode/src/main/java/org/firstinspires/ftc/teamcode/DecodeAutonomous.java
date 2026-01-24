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

@Autonomous(name = "DECODE Autonomous", preselectTeleOp = "DECODE TeleOp")
public class DecodeAutonomous extends OpMode {
    public enum InitState {
        HARDWARE,
        SELECT_ALLIANCE,
        COMPUTE_POSES,
        BUILD_PATHS,
        READY
    }

    public enum PathState {
        START,
        MOVE_SCORE_PRELOAD,
        SHOOT_PRELOAD,
        MOVE_GRAB_PICKUP_1,
        FINISH_GRAB_PICKUP_1,
        MOVE_SCORE_PICKUP_1,
        SHOOT_PICKUP_1,
        MOVE_GRAB_PICKUP_2,
        FINISH_GRAB_PICKUP_2,
        MOVE_SCORE_PICKUP_2,
        SHOOT_PICKUP_2,
        MOVE_GRAB_PICKUP_3,
        FINISH_GRAB_PICKUP_3,
        MOVE_PARK,
        END,
    }
    private InitState initState = InitState.HARDWARE;
    private PathState pathState;
    private double alliance;
    private String allianceName;
    private Follower follower;
    private Timer pathTimer;
    private final double SHOOTER_VELOCITY = 1550;
    private Pose startPose;
    private Pose shootPose;
    private Pose controlPose1;
    private Pose endPickupPose1;
    private Pose firstControlPose2;
    private Pose secondControlPose2;
    private Pose endPickupPose2;
    private Pose controlPose3;
    private Pose endPickupPose3;
    private Pose finalPose;
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scoreEnd;
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

                PIDFCoefficients coeffs = new PIDFCoefficients(15.0, 0.2, 2.5, 13.2);
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
                // Will move to nearSide = true block after far added
                startPose = new Pose(72 - (72 - 15.78) * alliance, 113.52, Math.toRadians(90 + 90 * alliance));
                shootPose = new Pose(72 - (72 - 48) * alliance, 96, Math.toRadians(90 + 45 * alliance));
                controlPose1 = new Pose(72 - (72 - 48) * alliance, 84);
                endPickupPose1 = new Pose(72 - (72 - 18) * alliance, 84, Math.toRadians(90 - 90 * alliance));
                firstControlPose2 = new Pose(72 - (72 - 48) * alliance, 60);
                secondControlPose2 = new Pose(72 - (72 - 36) * alliance, 60);
                endPickupPose2 = new Pose(72 - (72 - 18) * alliance, 60, Math.toRadians(90 - 90 * alliance));
                controlPose3 = new Pose(72 - (72 - 48) * alliance, 36);
                endPickupPose3 = new Pose(72 - (72 - 18) * alliance, 36, Math.toRadians(90 - 90 * alliance));
                finalPose = new Pose(72 - (72 - 27) * alliance, 72, Math.toRadians(90 + 90 * alliance));

                initState = InitState.BUILD_PATHS;
                break;

            case BUILD_PATHS:
                scorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, shootPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup1 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, controlPose1, endPickupPose1))
                        .setGlobalConstantHeadingInterpolation(endPickupPose1.getHeading())
                        .build();

                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose1, shootPose))
                        .setLinearHeadingInterpolation(endPickupPose1.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup2 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, firstControlPose2, secondControlPose2, endPickupPose2))
                        .setGlobalConstantHeadingInterpolation(endPickupPose2.getHeading())
                        .build();

                scorePickup2 = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose2, shootPose))
                        .setLinearHeadingInterpolation(endPickupPose2.getHeading(), shootPose.getHeading(), 0.75)
                        .build();

                grabPickup3 = follower.pathBuilder()
                        .addPath(new BezierCurve(shootPose, controlPose3, controlPose3, endPickupPose3))
                        .setGlobalConstantHeadingInterpolation(endPickupPose3.getHeading())
                        .build();

                scoreEnd = follower.pathBuilder()
                        .addPath(new BezierLine(endPickupPose3, finalPose))
                        .setLinearHeadingInterpolation(endPickupPose3.getHeading(), finalPose.getHeading(), 0.75)
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
    }

    @Override
    public void start() {
        setPathState(PathState.START);
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
            case START:
                follower.followPath(scorePreload, true);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(SHOOTER_VELOCITY * 7.0 / 15.0);
                }
                setPathState(PathState.MOVE_SCORE_PRELOAD);
                break;

            case MOVE_SCORE_PRELOAD:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.25) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.25) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3) {
                    break;
                }

                transferMotor.setPower(-0.2);
                follower.followPath(grabPickup1, 0.6, true);
                setPathState(PathState.MOVE_GRAB_PICKUP_1);
                break;

            case MOVE_GRAB_PICKUP_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.FINISH_GRAB_PICKUP_1);
                }
                break;

            case FINISH_GRAB_PICKUP_1:
                if (pathTimer.getElapsedTimeSeconds() < 0.5) {
                    break;
                }
                follower.followPath(scorePickup1, true);
                setPathState(PathState.MOVE_SCORE_PICKUP_1);
                break;

            case MOVE_SCORE_PICKUP_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_1);
                }
                break;

            case SHOOT_PICKUP_1:
                intakeMotor.setPower(0);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.25) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.25) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3) {
                    break;
                }

                transferMotor.setPower(-0.2);
                follower.followPath(grabPickup2, 0.6, true);
                setPathState(PathState.MOVE_GRAB_PICKUP_2);
                break;

            case MOVE_GRAB_PICKUP_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.FINISH_GRAB_PICKUP_2);
                }
                break;

            case FINISH_GRAB_PICKUP_2:
                if (pathTimer.getElapsedTimeSeconds() < 0.5) {
                    break;
                }
                follower.followPath(scorePickup2, true);
                setPathState(PathState.MOVE_SCORE_PICKUP_2);
                break;

            case MOVE_SCORE_PICKUP_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP_2);
                }
                break;

            case SHOOT_PICKUP_2:
                intakeMotor.setPower(0);
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 1.25) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 2.25) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 3) {
                    break;
                }

                transferMotor.setPower(-0.2);
                follower.followPath(grabPickup3, 0.6, true);
                setPathState(PathState.MOVE_GRAB_PICKUP_3);
                break;

            case MOVE_GRAB_PICKUP_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.FINISH_GRAB_PICKUP_3);
                }
                break;

            case FINISH_GRAB_PICKUP_3:
                if (pathTimer.getElapsedTimeSeconds() < 0.5) {
                    break;
                }
                follower.followPath(scoreEnd, true);
                setPathState(PathState.MOVE_PARK);
                break;

            case MOVE_PARK:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;

            case END:
                break;
        }
    }

    public void setPathState(PathState pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}