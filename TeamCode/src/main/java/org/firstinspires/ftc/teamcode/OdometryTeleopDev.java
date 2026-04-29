//package org.firstinspires.ftc.teamcode;
//
//// Imports
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//@TeleOp(name = "DECODE Teleop-Lumo")
//public class OdometryTeleopDev extends LinearOpMode {
//
//    // =======================
//    // Hardware
//    // =======================
//    private DcMotorEx LB, LF, RB, RF;
//    private DcMotorEx LS, RS, IN, TR;
//    private IMU imu;
//    private Limelight3A limelight;
//
//    // =======================
//    // Odometry State
//    // =======================
//    private double robotX = 0.0;
//    private double robotY = 0.0;
//
//    private double lastLB = 0, lastLF = 0, lastRB = 0, lastRF = 0;
//
//    private static final double TICKS_PER_REV = 537.7;
//    private static final double WHEEL_RADIUS = 0.048; // meters
//    private static final double METERS_PER_TICK =
//            (2 * Math.PI * WHEEL_RADIUS) / TICKS_PER_REV;
//
//    // =======================
//    // Goal Coordinates (meters)
//    // =======================
//    private static final double GOAL_X = -1.4827;
//    private static final double RED_GOAL_Y  =  1.4133;
//    private static final double BLUE_GOAL_Y = -1.4133;
//
//    // =======================
//    // Distance Fusion
//    // =======================
//    private double odometryDistance = -1;
//    private double limelightDistance = -1;
//    private double fusedDistance = -1;
//
//    // =======================
//    // Auto Aim
//    // =======================
//    private boolean autoAimEnabled = false;
//    private final double KP = 0.075;
//    private final double KF = 0.015;
//    private final double MAX_TURN = 0.75;
//
//    // =======================
//    // Shooter
//    // =======================
//    private double targetShooterVelocity = 0;
//    private double aimedShooterSpeed = 0;
//
//    private enum ShooterMode {
//        AUTO, BACK, ZERO, CLOSE, MID, FAR
//    }
//
//    private ShooterMode shooterMode = ShooterMode.ZERO;
//
//    // =======================
//    // OpMode
//    // =======================
//    @Override
//    public void runOpMode() {
//        initHardware();
//        waitForStart();
//
//        while (opModeIsActive()) {
//            updateOdometry();
//            updateOdometryDistance();
//            updateLimelightDistance();
//            fuseDistances();
//
//            drive();
//            intakeTransfer();
//            shooterLogic();
//            telemetryUpdate();
//        }
//    }
//
//    // =======================
//    // Odometry
//    // =======================
//    private void updateOdometry() {
//        double lb = LB.getCurrentPosition();
//        double lf = LF.getCurrentPosition();
//        double rb = RB.getCurrentPosition();
//        double rf = RF.getCurrentPosition();
//
//        double dLB = lb - lastLB;
//        double dLF = lf - lastLF;
//        double dRB = rb - lastRB;
//        double dRF = rf - lastRF;
//
//        lastLB = lb;
//        lastLF = lf;
//        lastRB = rb;
//        lastRF = rf;
//
//        double forwardTicks = (dLF + dRF + dLB + dRB) / 4.0;
//        double strafeTicks  = (-dLF + dRF + dLB - dRB) / 4.0;
//
//        double forward = forwardTicks * METERS_PER_TICK;
//        double strafe  = strafeTicks  * METERS_PER_TICK;
//
//        double heading = imu.getRobotYawPitchRollAngles()
//                .getYaw(AngleUnit.RADIANS);
//
//        robotX += forward * Math.cos(heading) - strafe * Math.sin(heading);
//        robotY += forward * Math.sin(heading) + strafe * Math.cos(heading);
//    }
//
//    private void updateOdometryDistance() {
//        double goalY = (DataPasser.currentAlliance == DataPasser.Alliance.RED)
//                ? RED_GOAL_Y : BLUE_GOAL_Y;
//
//        odometryDistance = Math.hypot(GOAL_X - robotX, goalY - robotY);
//    }
//
//    // =======================
//    // Limelight Distance
//    // =======================
//    private void updateLimelightDistance() {
//        limelightDistance = -1;
//
//        LLResult result = limelight.getLatestResult();
//        if (result == null || !result.isValid()) return;
//
//        Pose3D pose = result.getBotpose_MT2();
//        if (pose == null) return;
//
//        double goalY = (DataPasser.currentAlliance == DataPasser.Alliance.RED)
//                ? RED_GOAL_Y : BLUE_GOAL_Y;
//
//        limelightDistance = Math.hypot(
//                GOAL_X - pose.getPosition().x,
//                goalY - pose.getPosition().y
//        );
//    }
//
//    // =======================
//    // Distance Fusion
//    // =======================
//    private void fuseDistances() {
//        if (odometryDistance > 0 && limelightDistance > 0) {
//            fusedDistance = (odometryDistance + limelightDistance) / 2.0;
//        } else if (odometryDistance > 0) {
//            fusedDistance = odometryDistance;
//        } else {
//            fusedDistance = -1;
//        }
//    }
//
//    // =======================
//    // Drive + Auto Aim
//    // =======================
//    private void drive() {
//        double drive = -gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//
//        if (autoAimEnabled) {
//            LLResult result = limelight.getLatestResult();
//            if (result != null && result.isValid()) {
//                double tx = result.getTx();
//                turn = tx * KP + Math.signum(tx) * KF;
//                turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));
//            }
//        }
//
//        double lb = drive + turn - strafe;
//        double rb = drive - turn + strafe;
//        double lf = drive + turn + strafe;
//        double rf = drive - turn - strafe;
//
//        double scale = (gamepad1.left_stick_button) ? 2800 : 1200;
//
//        LB.setVelocity(lb * scale);
//        RB.setVelocity(rb * scale);
//        LF.setVelocity(lf * scale);
//        RF.setVelocity(rf * scale);
//
//        if (Math.abs(gamepad1.left_stick_x) > 0.1 ||
//                Math.abs(gamepad1.left_stick_y) > 0.1 ||
//                Math.abs(gamepad1.right_stick_x) > 0.1) {
//            autoAimEnabled = false;
//        }
//
//        if (gamepad2.right_trigger > 0.5) autoAimEnabled = true;
//    }
//
//    // =======================
//    // Intake / Transfer
//    // =======================
//    private void intakeTransfer() {
//        IN.setPower(-gamepad2.left_stick_y);
//        TR.setPower(gamepad2.left_bumper ? -1.0 : 0.6 * gamepad2.left_trigger - 0.2);
//    }
//
//    // =======================
//    // Shooter Logic
//    // =======================
//    private void shooterLogic() {
//        if (gamepad2.right_bumper) shooterMode = ShooterMode.AUTO;
//        if (gamepad2.a) shooterMode = ShooterMode.ZERO;
//        if (gamepad2.b) shooterMode = ShooterMode.CLOSE;
//        if (gamepad2.x) shooterMode = ShooterMode.MID;
//        if (gamepad2.y) shooterMode = ShooterMode.FAR;
//
//        if (fusedDistance > 0) {
//            aimedShooterSpeed =
//                    (0.00333057 * Math.pow(33.55221, fusedDistance)) +
//                            (191.64674 * fusedDistance) +
//                            971.0142;
//        }
//
//        switch (shooterMode) {
//            case AUTO:  targetShooterVelocity = aimedShooterSpeed; break;
//            case CLOSE: targetShooterVelocity = 1200; break;
//            case MID:   targetShooterVelocity = 1350; break;
//            case FAR:   targetShooterVelocity = 1700; break;
//            default:    targetShooterVelocity = 0;
//        }
//
//        targetShooterVelocity = Math.min(targetShooterVelocity, 2000);
//
//        LS.setVelocity(targetShooterVelocity * 7.0 / 14.6);
//        RS.setVelocity(targetShooterVelocity * 7.0 / 14.6);
//    }
//
//    // =======================
//    // Telemetry
//    // =======================
//    private void telemetryUpdate() {
//        telemetry.addData("Odo Distance", odometryDistance);
//        telemetry.addData("LL Distance", limelightDistance);
//        telemetry.addData("Fused Distance", fusedDistance);
//        telemetry.addData("Shooter Target", targetShooterVelocity);
//        telemetry.addData("Auto Aim", autoAimEnabled);
//        telemetry.update();
//    }
//
//    // =======================
//    // Hardware Init
//    // =======================
//    private void initHardware() {
//        LB = hardwareMap.get(DcMotorEx.class, "LB");
//        LF = hardwareMap.get(DcMotorEx.class, "LF");
//        RB = hardwareMap.get(DcMotorEx.class, "RB");
//        RF = hardwareMap.get(DcMotorEx.// Inside initHardware()
//                // limelight.start();
//        //shooterMode = ShooterMode.AUTO; // Change this from ZERO to AUTOclass, "RF");
//
//        //LS = hardwareMap.get(DcMotorEx.class, "LS");
//        //RS = hardwareMap.get(DcMotorEx.class, "RS");
//        //IN = hardwareMap.get(DcMotorEx.class, "IN");
//        //TR = hardwareMap.get(DcMotorEx.class, "TR");
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        limelight = hardwareMap.get(Limelight3A.class, "LM");
//        limelight.start();
//
//        LB.setDirection(DcMotor.Direction.REVERSE);
//        LF.setDirection(DcMotor.Direction.REVERSE);
//        LS.setDirection(DcMotor.Direction.REVERSE);
//        RS.setDirection(DcMotor.Direction.REVERSE);
//        PIDFCoefficients shooterPID = new PIDFCoefficients(45, 0.02, 2.5, 13.2);
//        LS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
//        RS.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
//    }
//}
