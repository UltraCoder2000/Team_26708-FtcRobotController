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

@Autonomous(name = "Decode Autonomous (Red)")
public class DecodeAutonomousRed extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private final double SHOOTER_VELOCITY = 1500;
    private final Pose startPose = new Pose(118.72,132.75, Math.toRadians(35.954));
    private final Pose shootPose = new Pose(96, 96, Math.toRadians(45));
    private final Pose controlPose = new Pose(84, 84);
    private final Pose startPickupPose = new Pose(96, 84, Math.toRadians(180));
    private final Pose endPickupPose = new Pose(126, 84, Math.toRadians(180));
    private final Pose finalPose = new Pose(96, 60, Math.toRadians(180));
    private PathChain scorePreload, grabPickup, scorePickup, scoreEnd;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;
    private DcMotorEx[] shooters;

    @Override
    public void init() {
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

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
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

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, controlPose, startPickupPose))
                .addPath(new BezierLine(startPickupPose, endPickupPose))
                .setGlobalConstantHeadingInterpolation(endPickupPose.getHeading())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(endPickupPose, shootPose))
                .setLinearHeadingInterpolation(endPickupPose.getHeading(), shootPose.getHeading(), 0.75)
                .build();

        scoreEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, finalPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), finalPose.getHeading(), 0.75)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(SHOOTER_VELOCITY * 7/15);
                }
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2.3) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 4) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 5) {
                    break;
                }

                transferMotor.setPower(0);
                follower.followPath(grabPickup, 0.4, true);
                setPathState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                transferMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 2.3) {
                    break;
                }

                intakeMotor.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() < 4) {
                    break;
                }

                intakeMotor.setPower(1);
                if (pathTimer.getElapsedTimeSeconds() < 5) {
                    break;
                }

                transferMotor.setPower(0);
                intakeMotor.setPower(0);
                for (DcMotorEx motor : shooters) {
                    motor.setVelocity(0);
                }
                follower.followPath(scoreEnd, true);
                setPathState(6);
                break;

            case 6:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}