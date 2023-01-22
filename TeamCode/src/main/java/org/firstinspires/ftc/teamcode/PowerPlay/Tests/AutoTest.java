package org.firstinspires.ftc.teamcode.PowerPlay.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;

// Put the cone in the blue robot butt and point it to where you want to go
@Autonomous (name = "AutoTest")
public class AutoTest extends LinearOpMode {
    PowerPlayRobot ewok;
    @Override
    public void runOpMode() throws InterruptedException {
        //we have landed on a moon of Endor.
        ewok = new PowerPlayRobot(this);
        ewok.initHardware();

        waitForStart();
        if(opModeIsActive()) {
            ewok.rotate(90);
            telemetry.update();
            sleep(5000);
        }
    }
}