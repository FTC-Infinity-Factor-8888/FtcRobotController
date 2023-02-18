package org.firstinspires.ftc.teamcode.PowerPlay.Autonomous;


import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags.SignalLocation.ZONE_1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags.SignalLocation.ZONE_2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags.SignalLocation.ZONE_3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags.SignalDetector;
import org.firstinspires.ftc.teamcode.PowerPlay.Vision.AprilTags.SignalLocation;

// Put the on the RIGHT side of the robot
@Autonomous (name = "VisionRed-ConeOnRight")
public class DriveVisionRed extends LinearOpMode {
    PowerPlayRobot ewok;
    SignalDetector vision;
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new PowerPlayRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()) {
            SignalLocation zone = ewok.getZoneOfInterest();

            ewok.strafe(29);
            ewok.strafe(-29);
            if (zone == ZONE_1) {
                ewok.drive(30);
                ewok.strafe(24);
            }
            else if (zone == ZONE_2) {
                ewok.drive(34);
            }
            else if (zone == ZONE_3) {
                ewok.drive(28);
                ewok.strafe(-24);
            }
            else {
                ewok.strafe(29);
            }
        }
    }
}