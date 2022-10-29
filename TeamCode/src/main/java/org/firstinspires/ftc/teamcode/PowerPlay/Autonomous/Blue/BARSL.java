package org.firstinspires.ftc.teamcode.PowerPlay.Autonomous.Blue;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;

@Autonomous (name = "BARSL")
public class BARSL extends LinearOpMode {
    PowerPlayRobot ewok;
    //start line 1 is the closest to the warehouse for both alliances (lined up with middle barcode)
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new PowerPlayRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()) {
            ewok.drive(37);
        }
    }
}