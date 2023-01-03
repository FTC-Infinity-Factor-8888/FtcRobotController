package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.IntakePosition;

@TeleOp(name = "GrabberTest")
public class GrabberTest extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        // Declaring the former values of the buttons, so we can tell if they changed.
        // Note: Currently no such variables
        boolean intake;
        boolean outtake;
        boolean neutral;
        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initHardware();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                intake = gamepad2.a;
                outtake = gamepad2.y;
                neutral = gamepad2.x;
                if (intake && !outtake && !neutral) {
                    r2.intakeMotor(IntakePosition.IN);
                }
                else if (!intake && outtake && !neutral) {
                    r2.intakeMotor(IntakePosition.OUT);
                }
                telemetry.update();
            }
        }
    }
}
