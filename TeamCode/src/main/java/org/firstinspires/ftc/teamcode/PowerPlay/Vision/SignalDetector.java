package org.firstinspires.ftc.teamcode.PowerPlay.Vision;

import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//todo potentially import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class SignalDetector extends LinearOpMode {
    PowerPlayRobot ewok;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;

    final static int ID_TAG_ZONE_1 = 18; // Tag ID 18 from the 36h11 family
    final static int ID_TAG_ZONE_2 = 0; // Tag ID 0 from the 36h11 family
    final static int ID_TAG_ZONE_3 = 10; // Tag ID 10 from the 36h11 family
    public SignalLocation zoneOfInterest = null;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {   //todo: May have to change this vvvvvv
        PowerPlayRobot ewok = new PowerPlayRobot(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (currentDetections != null && currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_ZONE_1) {
                        tagOfInterest = tag;
                        tagFound = true;
                        zoneOfInterest = ZONE_1;
                        break;
                    }
                    if (tag.id == ID_TAG_ZONE_2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        zoneOfInterest = ZONE_2;
                        break;
                    }
                    if (tag.id == ID_TAG_ZONE_3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        zoneOfInterest = ZONE_3;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addData("TargetZone", zoneOfInterest);
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");

                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

    }
}