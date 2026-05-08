package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Daniel_AutoDriveToAprilTagOmni", group = "Concept")
public class Daniel_AutoDriveToAprilTagOmni extends LinearOpMode {

    private static final boolean USE_WEBCAM    = true;
    private static final int    DESIRED_TAG_ID = -1; // -1 = any tag

    // Drive motors
    private DcMotor frontLeftDrive  = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor backRightDrive  = null;

    // Launcher motor  ← must be named "launcher" in your robot config
    private DcMotor launcher = null;

    // Vision
    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addData("Status", "Ready — waiting for START");
        telemetry.update();
        waitForStart();

        // ── Phase 1: Drive forward for 0.5 seconds ───────────────────────────
        moveRobot(0.5, 0, 0);
        sleep(500);
        moveRobot(0, 0, 0);

        // ── Phase 2: Slow left turn — stop immediately on target detect ───────
        boolean targetDetected = scanAndTurn();

        // ── Phase 3: If target was found, spin up and launch indefinitely ─────
        if (targetDetected) {
            launchBall();
        }
    }

    /**
     * Slowly turns left for up to 2 seconds.
     * Polls the AprilTag sensor every loop — the moment a target is seen,
     * it stops turning, rumbles the gamepad, and returns true.
     *
     * @return true if a target was found, false if the turn timed out
     */
    private boolean scanAndTurn() {
        long turnStart = System.currentTimeMillis();
        final long TURN_DURATION_MS = 2000;

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - turnStart;

            // ── Check for target ──────────────────────────────────────────────
            AprilTagDetection found = getTarget();

            if (found != null) {
                // Stop turning immediately
                moveRobot(0, 0, 0);

                // ── Notify driver via gamepad rumble and telemetry ────────────
                gamepad1.rumble(1.0, 1.0, 500);   // both motors, 500 ms buzz
                gamepad2.rumble(1.0, 1.0, 500);

                telemetry.addLine("!! TARGET ACQUIRED !!");
                telemetry.addData("AprilTag ID",  found.id);
                telemetry.addData("Range",    "%.1f in", found.ftcPose.range);
                telemetry.addData("Bearing",  "%.1f deg", found.ftcPose.bearing);
                telemetry.addData("Launcher", "SPINNING UP...");
                telemetry.update();

                return true; // hand off to launchBall()
            }

            // ── Still turning left ────────────────────────────────────────────
            if (elapsed >= TURN_DURATION_MS) {
                moveRobot(0, 0, 0);
                telemetry.addData("Turn", "Complete — no target found");
                telemetry.update();
                return false;
            }

            moveRobot(0, 0, 0.25); // slow left turn

            telemetry.addData("Turning left", "%.1f s remaining",
                    (TURN_DURATION_MS - elapsed) / 1000.0);
            telemetry.addData("Target", "Searching...");
            telemetry.update();
        }

        moveRobot(0, 0, 0);
        return false;
    }

    /**
     * Spins up the launcher to maximum power and keeps it running
     * until the OpMode ends. Telemetry updates continuously.
     */
    private void launchBall() {
        launcher.setPower(1.0); // maximum speed — runs indefinitely

        while (opModeIsActive()) {
            telemetry.addLine("!! LAUNCHING !!");
            telemetry.addData("Launcher Power", "100%  (MAX)");
            telemetry.addData("Status", "Running until OpMode ends");
            telemetry.update();
        }

        // OpMode ending — safe shutdown
        launcher.setPower(0);
        moveRobot(0, 0, 0);
    }

    /**
     * Checks the current AprilTag detections and returns the first
     * matching target, or null if none is visible.
     */
    private AprilTagDetection getTarget() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null &&
                (DESIRED_TAG_ID < 0 || d.id == DESIRED_TAG_ID)) {
                return d;
            }
        }
        return null;
    }

    // ── Hardware init ─────────────────────────────────────────────────────────

    private void initHardware() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rightBack");
        launcher        = hardwareMap.get(DcMotor.class, "launcher");

        frontLeftDrive .setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive  .setDirection(DcMotor.Direction.REVERSE);
        backRightDrive .setDirection(DcMotor.Direction.FORWARD);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // ── Mecanum drive helper ──────────────────────────────────────────────────

    public void moveRobot(double x, double y, double yaw) {
        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                              Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeftDrive .setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive  .setPower(bl);
        backRightDrive .setPower(br);
    }

    // ── Vision init ───────────────────────────────────────────────────────────

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = USE_WEBCAM
            ? new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag).build()
            : new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(aprilTag).build();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        while (!isStopRequested() &&
               visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (!isStopRequested()) {
            ExposureControl ec = visionPortal.getCameraControl(ExposureControl.class);
            if (ec.getMode() != ExposureControl.Mode.Manual) {
                ec.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            ec.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            visionPortal.getCameraControl(GainControl.class).setGain(gain);
            sleep(20);
        }
    }
}
