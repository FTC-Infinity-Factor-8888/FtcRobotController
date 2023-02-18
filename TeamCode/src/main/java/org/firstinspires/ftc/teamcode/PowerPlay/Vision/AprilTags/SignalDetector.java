package org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class SignalDetector {

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

    public void initalizeVision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    /*
    public SignalLocation getSignalLocation() {
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
        }
        return zoneOfInterest;
    }
     */


    //TODO: Dave review
    public SignalLocation getZoneOfInterest(){
        SignalLocation zoneOfInterest = aprilTagDetectionPipeline.getZoneDetections();
        return zoneOfInterest;
    }

}