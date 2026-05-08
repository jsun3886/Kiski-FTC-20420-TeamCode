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

    private static final boolean USE_WEBCAM  = true;
    private static final int    DESIRED_TAG_ID = -1; // -1 = any tag

    // Drive motors
    private DcMotor frontLeftDrive  = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor backRightDrive  = null;

    // Launcher motor  ← add "launcher" to your robot config with this exact name
    private DcMotor launcher = null;

    // Vision
    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        // ── Phase 1: Drive forward for 0.5 seconds ──────────────────────────
        moveRobot(0.5, 0, 0);
        sleep(500);
        moveRobot(0, 0, 0);

        // ── Phase 2: Slow left turn for 2 seconds (less than 90°) ───────────
        moveRobot(0, 0, 0.25); // small turn power → stays well under 90°
        sleep(2000);
        moveRobot(0, 0, 0);

        // ── Phase 3: Scan for AprilTag and fire if found ─────────────────────
        boolean firedAlready = false;

        while (opModeIsActive() && !firedAlready) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection d : detections) {
                if (d.metadata != null &&
                    (DESIRED_TAG_ID < 0 || d.id == DESIRED_TAG_ID)) {

                    // Target found — fire launcher at full power
                    launcher.setPower(1.0);
                    sleep(1500);          // run launcher long enough to release the ball
                    launcher.setPower(0);
                    firedAlready = true;

                    telemetry.addData("Target", "FOUND — ID %d", d.id);
                    telemetry.addData("Launcher", "FIRED");
                    telemetry.update();
                    break;
                }
            }

            if (!firedAlready) {
                telemetry.addData("Target", "Searching...");
                telemetry.update();
            }
        }

        // Stop all motion when done
        moveRobot(0, 0, 0);
    }

    // ── Hardware init ────────────────────────────────────────────────────────

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

    // ── Mecanum drive helper ─────────────────────────────────────────────────
    // x = forward/back, y = strafe left/right, yaw = rotation

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

    // ── Vision init ──────────────────────────────────────────────────────────

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
