package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Tuner")
@Disabled
public class PinpointTuner extends OpMode {

    private static final String PINPOINT_NAME = "PC";

    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx leftBackDrive, leftFrontDrive, rightBackDrive, rightFrontDrive;

    // drive speed coefficients
    private static final double SLOW_MODE_COEFF = 1050.0;
    private static final double FAST_MODE_COEFF = 2800.0;

    // variables for chassis control
    private double scale = 0.0;
    private double velLB = 0.0;
    private double velRB = 0.0;
    private double velLF = 0.0;
    private double velRF = 0.0;

    
    // offsets in mm
    private double xOffsetMm = 52.0;
    private double yOffsetMm = 156.0;

    // encoder directions
    private boolean xEncoderReversed = true;
    private boolean yEncoderReversed = false;

    // step sizes
    private final double coarseStep = 10.0;   // mm
    private final double fineStep   = 1.0;    // mm

    // toggle for coarse / fine
    private boolean coarseMode = true;

    // button edge detection
    private boolean lastDpadUp      = false;
    private boolean lastDpadDown    = false;
    private boolean lastDpadLeft    = false;
    private boolean lastDpadRight   = false;
    private boolean lastY           = false;
    private boolean lastA           = false;
    private boolean lastX           = false;
    private boolean lastB           = false;
    private boolean lastLeftBumper  = false;
    private boolean lastRightBumper = false;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        leftBackDrive = hardwareMap.get(DcMotorEx.class, "LB");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "LF");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "RB");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "RF");

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Zero everything at initialization with robot stationary
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void loop() {
        // Request fresh data once per loop
        pinpoint.update();

        driveChassis();
        handleTuning();
        reportTelemetry();
    }

    private void driveChassis() {
        double drive = -gamepad1.left_stick_y;   // forward/back
        double strafe = gamepad1.left_stick_x;   // left/right
        double turn = gamepad1.right_stick_x;    // rotation

        // Compute raw motor power values
        double LB = drive + turn - strafe;
        double RB = drive - turn + strafe;
        double LF = drive + turn + strafe;
        double RF = drive - turn - strafe;

        // For smoother scaling, square the average of the absolute values of the raw numbers above
        double avgAbs = (Math.abs(LB) + Math.abs(RB) + Math.abs(LF) + Math.abs(RF)) / 4.0;
        scale = avgAbs * avgAbs; // squared average magnitude

        // Choose drive speed coefficient based on left stick press
        double coeff = (gamepad1.left_stick_button || gamepad1.right_stick_button)
                        ? FAST_MODE_COEFF
                        : SLOW_MODE_COEFF;

        // Final velocities (apply scale and coefficient)
        velLB = LB * scale * coeff;
        velRB = RB * scale * coeff;
        velLF = LF * scale * coeff;
        velRF = RF * scale * coeff;

        // Set motor velocities
        leftBackDrive.setVelocity(velLB);
        rightBackDrive.setVelocity(velRB);
        leftFrontDrive.setVelocity(velLF);
        rightFrontDrive.setVelocity(velRF);
    }

    private void handleTuning() {
        // Y: toggle coarse / fine mode
        if (gamepad1.y && !lastY) {
            coarseMode = !coarseMode;
        }

        double step = coarseMode ? coarseStep : fineStep;

        // X offset: left/right D-pad
        if (gamepad1.dpad_right && !lastDpadRight) {
            xOffsetMm += step;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            xOffsetMm -= step;
        }

        // Y offset: up/down D-pad
        if (gamepad1.dpad_up && !lastDpadUp) {
            yOffsetMm += step;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            yOffsetMm -= step;
        }

        // Apply offsets to Pinpoint each loop
        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);

        // Toggle X encoder direction: left bumper
        // Toggle Y encoder direction: right bumper
        if (gamepad1.left_bumper && !lastLeftBumper) {
            xEncoderReversed = !xEncoderReversed;
        }        
        if (gamepad1.right_bumper && !lastRightBumper) {
            yEncoderReversed = !yEncoderReversed;
        }
        pinpoint.setEncoderDirections(
            xEncoderReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                             : GoBildaPinpointDriver.EncoderDirection.FORWARD,
            yEncoderReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                             : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // A: reset pose + IMU (robot must be still)
        if (gamepad1.a && !lastA) {
            pinpoint.resetPosAndIMU();
        }

        // X: recalibrate IMU only (keeps current pose, robot must be still)
        if (gamepad1.x && !lastX) {
            pinpoint.recalibrateIMU();
        }

        // B: zero just the estimated pose (keep IMU calibration)
        if (gamepad1.b && !lastB) {
            // Get current heading (in degrees or radians — just match the AngleUnit below)
            Pose2D currentPose = pinpoint.getPosition();
            double currentHeadingDeg = currentPose.getHeading(AngleUnit.DEGREES);

            // Build a new Pose2D at (0, 0) with the same heading
            Pose2D zeroedPose =
                    new Pose2D(
                            DistanceUnit.MM,
                            0.0,
                            0.0,
                            AngleUnit.DEGREES,
                            currentHeadingDeg
                    );

            // Apply it to Pinpoint
            pinpoint.setPosition(zeroedPose);
        }

        // Used for change detection
        lastDpadUp      = gamepad1.dpad_up;
        lastDpadDown    = gamepad1.dpad_down;
        lastDpadLeft    = gamepad1.dpad_left;
        lastDpadRight   = gamepad1.dpad_right;
        lastY           = gamepad1.y;
        lastA           = gamepad1.a;
        lastX           = gamepad1.x;
        lastB           = gamepad1.b;
        lastLeftBumper  = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }

    private void reportTelemetry() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
    
        // Pose in mm and degrees
        Pose2D pose = pinpoint.getPosition();
        double xMm   = pose.getX(DistanceUnit.MM);
        double yMm   = pose.getY(DistanceUnit.MM);
        double hDeg  = pose.getHeading(AngleUnit.DEGREES);
    
        // Velocity in mm/sec and deg/sec
        double vxMm  = pinpoint.getVelX(DistanceUnit.MM);
        double vyMm  = pinpoint.getVelY(DistanceUnit.MM);
        double vhDeg = pinpoint.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized());
    
        telemetry.addLine("PINPOINT TUNER");
        telemetry.addData("Status", status);
    
        telemetry.addLine();
        telemetry.addLine("=== Offset Tuning ===");
        telemetry.addData("Offsets (mm)", "X: %.1f   Y: %.1f", xOffsetMm, yOffsetMm);
        telemetry.addData("Step Mode", "%s (Y to toggle)", coarseMode ? "COARSE" : "FINE");
        telemetry.addData("Step Size (mm)", "%.1f", coarseMode ? coarseStep : fineStep);
    
        telemetry.addLine();
        telemetry.addLine("=== Encoder Directions ===");
        telemetry.addData("X Encoder", xEncoderReversed ? "REVERSED" : "NORMAL");
        telemetry.addData("Y Encoder", yEncoderReversed ? "REVERSED" : "NORMAL");
    
        telemetry.addLine();
        telemetry.addLine("=== Pose ===");
        telemetry.addData("Position (mm, deg)", "X: %.1f   Y: %.1f   H: %.1f",
                xMm, yMm, hDeg);
    
        telemetry.addLine();
        telemetry.addLine("=== Velocity ===");
        telemetry.addData("Velocity (mm/s, deg/s)", "X: %.1f   Y: %.1f   H: %.1f",
                vxMm, vyMm, vhDeg);
    
        telemetry.addLine();
        telemetry.addLine("=== Drive Info ===");
        telemetry.addData("Scale", "%.3f", scale);
        telemetry.addData("LB Intended Vel", velLB);
        telemetry.addData("RB Intended Vel", velRB);
        telemetry.addData("LF Intended Vel", velLF);
        telemetry.addData("RF Intended Vel", velRF);
        telemetry.addData("Drive Mode",
                (gamepad1.left_stick_button || gamepad1.right_stick_button)
                        ? "FAST (2800)"
                        : "SLOW (1050)");
    
        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("  D-pad L/R : X offset -/+");
        telemetry.addLine("  D-pad D/U : Y offset -/+");
        telemetry.addLine("  Y         : toggle coarse/fine step");
        telemetry.addLine("  A         : resetPosAndIMU (zero pose + IMU)");
        telemetry.addLine("  X         : recalibrateIMU");
        telemetry.addLine("  B         : zero pose only");
        telemetry.addLine("  L Bumper  : toggle X encoder direction");
        telemetry.addLine("  R Bumper  : toggle Y encoder direction");
        telemetry.addLine("  LS/RS Btn : FAST drive mode");
        
        telemetry.update();
    }
}
