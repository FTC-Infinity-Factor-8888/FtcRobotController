package org.firstinspires.ftc.teamcode.PowerPlay.Autonomous;


import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation.ZONE_3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalDetector;
import org.firstinspires.ftc.teamcode.PowerPlay.Vision.SignalLocation;

// Put the cone in the blue robot butt and point it to where you want to go
@Autonomous (name = "Vision")
public class Vision extends LinearOpMode {
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

            ewok.strafe(-26);
            ewok.strafe(26);
            if (zone == ZONE_1) {
                ewok.drive(28);
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
                ewok.strafe(20);
            }
        }
    }
}