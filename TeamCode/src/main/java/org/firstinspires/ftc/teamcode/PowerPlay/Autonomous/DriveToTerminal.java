package org.firstinspires.ftc.teamcode.PowerPlay.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;

// Blue alliance, Left side line, going right
@Autonomous (name = "TerminalDrive")
public class DriveToTerminal extends LinearOpMode {
    PowerPlayRobot ewok;
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