
//Correct Code for TeleOp
 package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


@TeleOp(name = "Decode TeleOp LumoJUMP")
public class DecodeTeleOp extends LinearOpMode {
    // Motors
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;
    private DcMotorEx leftShooter, rightShooter, intakeMotor, transferMotor;

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    // Constants
    private double targetX;
    private double targetY;
    private final double KP = 0.020; //proportional constant
    private final double KF = 0.015; //proportional constant
    private final double MAX_TURN_OUTPUT = 0.75;

    // Variables
    private double horizontalDistance = -1;
    private double yaw = 0;
    private boolean autoAimEnabled = false;
    private double targetShooterVelocity = 0;
    private com.qualcomm.robotcore.util.ElapsedTime matchTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private boolean autoParking = false;
    private Follower follower; // This handles the PedroPathing movement
    private boolean isFullyAimed = false;
    private double autoFireStartTime = -1;
    private double aimedShooterSpeed = 0;
    private enum DriveSpeed {
        SLOW,
        FAST,
        ULTRA
    }
    private DriveSpeed driveSpeed = DriveSpeed.SLOW;
    private enum ShooterMode {
        AUTO,
        BACK,
        ZERO,
        CLOSE,
        MID,
        FAR
    }

    ShooterMode shooterMode = ShooterMode.AUTO;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        initHardware();
        telemetry.addData("Status", "Initialized, waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            limelightLogic();
            drive();
            intakeAndTransfer();
            shooterLogic();
            updateTelemetry();
        }
    }

    private void limelightLogic() {
        // Autonomous must be initialized while facing opposite alliance wall for auto-aiming to work
        targetX = -1.4827; // April tag position
        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            // Adjusts yaw sent to Limelight as its coordinate system is different
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 90;
            targetY = 1.4133; // Red goal
        } else {
            yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90;
            targetY = -1.4133; // Blue goal
        }

        // Normalizes yaw values
        while (yaw > 180)   yaw -= 360;
        while (yaw <= -180) yaw += 360;

        // Updates Limelight values
        limelight.updateRobotOrientation(yaw);
        LLResult result = limelight.getLatestResult();

        // If result valid, calculates position of bot and distance to goal
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double dx = targetX - botpose.getPosition().x;
                double dy = targetY - botpose.getPosition().y;
                horizontalDistance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            }
        } else {
            horizontalDistance = -1; // Placeholder value as no valid result
        }

        // Redundancy in case of Autonomous program failing
        if (gamepad1.dpad_left) {
            DataPasser.currentAlliance = DataPasser.Alliance.BLUE;
            imu.resetYaw();
            limelight.pipelineSwitch(0);
        }
        if (gamepad1.dpad_right) {
            DataPasser.currentAlliance = DataPasser.Alliance.RED;
            imu.resetYaw();
            limelight.pipelineSwitch(1);
        }

        // Disables chassis auto-aim if gamepad 1 touches sticks
        if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
            autoAimEnabled = false;
        }

        // Overrides above case
        // Overrides above case
        if (gamepad2.right_trigger > 0.5) { // Use > 0.5 for triggers
            autoAimEnabled = true;
        }

    }

    private void drive() {
        // 1. Determine Base Speed Mode
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            driveSpeed = DriveSpeed.FAST;
            if (gamepad1.a) {driveSpeed = DriveSpeed.ULTRA;
            }
        } else {
            driveSpeed = DriveSpeed.SLOW;
        }

        // 2. Set Multipliers
        double speedMultiplier;
        switch (driveSpeed) {
            case ULTRA: speedMultiplier = 1.0;  break;
            case FAST:  speedMultiplier = 0.75; break;
            case SLOW:
            default:    speedMultiplier = 0.45; break;
        }

        // 3. Get Joystick Inputs
        double drive = -gamepad1.left_stick_y * speedMultiplier;
        double strafe = gamepad1.left_stick_x * speedMultiplier;
        double turn = gamepad1.right_stick_x * speedMultiplier;

        LLResult result = limelight.getLatestResult();

        // 4. Handle Auto-Aiming (Rotation Hijack)
        if (autoAimEnabled && result != null && result.isValid()) {
            double tx = result.getTx(); // The horizontal offset from crosshair

            // DEADBAND: If error is less than 1.0 degree, stop turning
            if (Math.abs(tx) < 1.0) {
                turn = 0;
                // Start the fire timer if we haven't already
                if (autoFireStartTime == -1) {
                    isFullyAimed = true;
                    autoFireStartTime = getRuntime();
                }
            } else {
                // Calculation: Error * Proportional + Constant Friction Kick
                // We use Math.signum(tx) to ensure KF always pushes TOWARDS the target
                turn = (tx * KP) + (Math.signum(tx) * KF);

                // Limit the turn speed so it doesn't whip around too fast
                turn = Math.max(-MAX_TURN_OUTPUT, Math.min(MAX_TURN_OUTPUT, turn));

                isFullyAimed = false;
                autoFireStartTime = -1;
            }
        }

        // 5. Mecanum Motor Calculations
        double p1 = drive + strafe + turn; // LF
        double p2 = drive - strafe + turn; // LB
        double p3 = drive - strafe - turn; // RF
        double p4 = drive + strafe - turn; // RB

        // Normalize powers if any exceed 1.0
        double max = Math.max(1.0, Math.max(Math.abs(p1), Math.max(Math.abs(p2), Math.max(Math.abs(p3), Math.abs(p4)))));

        leftFrontDrive.setPower(p1 / max);
        leftBackDrive.setPower(p2 / max);
        rightFrontDrive.setPower(p3 / max);
        rightBackDrive.setPower(p4 / max);
    }


    private void intakeAndTransfer() {
        // --- AUTO-FIRE SEQUENCE ---
        if (isFullyAimed && autoFireStartTime != -1) {
            double timeElapsed = getRuntime() - autoFireStartTime;

            // Fire for 3 seconds after being fully aimed
            if (timeElapsed < 3.0) {
                intakeMotor.setPower(0.5);    // Spin intake forward
                transferMotor.setPower(0.7); // Spin transfer to shoot
            } else {
                // After 3 seconds, stop the sequence and reset
                isFullyAimed = false;
                autoFireStartTime = -1;
                // Optional: Automatically disable auto-aim so the driver has full control back
                autoAimEnabled = false;
            }
        } else {
            // --- MANUAL CONTROLS ---
            // This runs only if the auto-fire sequence isn't active
            intakeMotor.setPower(-gamepad2.left_stick_y);
            if (gamepad2.left_bumper) {
                transferMotor.setPower(-1.0);
            } else {
                transferMotor.setPower(0.6 * gamepad2.left_trigger - 0.2);
            }
        }
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }


    private void shooterLogic() {
        // 1. Manual Overrides (Driver can still force a mode)
        if (gamepad2.a)  shooterMode = ShooterMode.ZERO;
        if (gamepad2.b)  shooterMode = ShooterMode.CLOSE;
        if (gamepad2.x)  shooterMode = ShooterMode.MID;
        if (gamepad2.y)  shooterMode = ShooterMode.FAR;
        if (gamepad2.right_stick_button) shooterMode = ShooterMode.BACK;
        if (gamepad2.right_bumper) shooterMode = ShooterMode.AUTO;

        // 2. AUTOMATIC HIJACK: If Limelight sees a tag, force AUTO mode
        if (horizontalDistance > 0) {
            shooterMode = ShooterMode.AUTO;

            // Tuning Constants
            double A = 25.0;
            double B = 140.0;
            double C = 960.0;

            // --- ADDED OFFSET ---
            // Add a flat 50-100 ticks/sec to "over-shoot" the target slightly.
            // Or use a multiplier like 1.05 for 5% extra power.
            double extraPower = 1.02;

            aimedShooterSpeed = ((A * Math.pow(horizontalDistance, 2)) + (B * horizontalDistance) + C) * extraPower;
        }

        // 3. Determine final velocity based on mode
        switch (shooterMode) {
            case AUTO:
                targetShooterVelocity = (horizontalDistance > 0) ? aimedShooterSpeed : 0;
                break;
            case CLOSE: targetShooterVelocity = 1200; break;
            case MID:   targetShooterVelocity = 1350; break;
            case FAR:   targetShooterVelocity = 1700; break;
            case BACK:  targetShooterVelocity = -500; break;
            default:    targetShooterVelocity = 0;    break;
        }

        // 4. Clamp and Apply Gear Ratio
        targetShooterVelocity = Math.min(targetShooterVelocity, 2100);

        // This math is correct ONLY if motor has 15T and wheel has 7T
        double scaledVelocity = targetShooterVelocity * 7.0 / 15.0;

        leftShooter.setVelocity(scaledVelocity);
        rightShooter.setVelocity(scaledVelocity);
    }

//    private void handleEndgameAutoPark() {
//        if (autoParking) return;
//        if (matchTimer.seconds() < 90) return;
//
//        if (gamepad1.x) {
//            autoParking = true;
//
//            Pose parkPose = (alliance == Alliance.RED)
//                    ? Pose.fromField(58, 12, Math.toRadians(180))
//                    : Pose.fromField(58, -12, Math.toRadians(180));
//
//            follower.setTargetPose(parkPose);
//        }
//    }
    private void initHardware() {
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "LB");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LF");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "RB");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "RF");

        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "IN");
        transferMotor = hardwareMap.get(DcMotorEx.class, "TR");

        DcMotor[] drives = {leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive};
        for (DcMotor motor : drives) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx[] loaders = {intakeMotor, transferMotor};
        for (DcMotorEx motor : loaders) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        PIDFCoefficients coeffs = new PIDFCoefficients(45.0, 0.02, 2.5, 13.2);
        DcMotorEx[] shooters = {leftShooter, rightShooter};
        for (DcMotorEx motor : shooters) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
        }

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "LM");
        if (DataPasser.currentAlliance == DataPasser.Alliance.RED) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }
        limelight.start();
    }

    private void updateTelemetry() {
        telemetry.addData("Target Found", (horizontalDistance == -1) ? "NO" : "YES");
        telemetry.addData("Distance to Target", "%.2f", horizontalDistance);
        telemetry.addData("Shooter Velocity", targetShooterVelocity);
        telemetry.addData("Actual Left Velocity", leftShooter.getVelocity() * 15/7);
        telemetry.addData("Actual Right Velocity", rightShooter.getVelocity() * 15/7);
        telemetry.addData("Auto-Aim Active", autoAimEnabled);
        telemetry.addData("SHOOTER MODE:", shooterMode);
        telemetry.addData("Dist", aimedShooterSpeed);
        telemetry.addData("Shooter Speed", targetShooterVelocity);


        // Updated Drive Mode Telemetry
        String modeName = driveSpeed.toString();
        double powerPercent = (driveSpeed == DriveSpeed.ULTRA) ? 100 : (driveSpeed == DriveSpeed.FAST ? 75 : 45);

        telemetry.addData("Drive Mode", "%s (%.0f%%)", modeName, powerPercent);
        telemetry.update();
    }


}
