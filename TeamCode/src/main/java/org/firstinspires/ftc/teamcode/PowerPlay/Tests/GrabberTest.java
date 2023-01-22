package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.IntakePosition;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.WristPosition;

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
        boolean wristUp;
        boolean wristDown;
        boolean wristMiddle;
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
                if (intake && !outtake) {
                    r2.intakeMotor(IntakePosition.IN);
                }
                else if (!intake && outtake) {
                    r2.intakeMotor(IntakePosition.OUT);
                }

                wristUp = gamepad2.dpad_up;
                wristDown = gamepad2.dpad_down;
                wristMiddle = gamepad2.dpad_right;
                if (wristUp && !wristDown && !wristMiddle) {
                    r2.wristMotor(WristPosition.UP);
                }
                else if (!wristUp && wristDown && !wristMiddle) {
                    r2.wristMotor(WristPosition.DOWN);
                }
                else if (!wristUp && !wristDown && wristMiddle) {
                    r2.wristMotor(WristPosition.MIDDLE);
                }
                telemetry.update();
            }
        }
    }
}
