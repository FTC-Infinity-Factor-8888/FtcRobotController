package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;

@TeleOp(name = "TeleOpMain")
public class TeleOpTest extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        boolean liftUp;
        boolean liftDown;

        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initTestHardware();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                liftUp = gamepad1.right_bumper;
                liftDown = gamepad1.left_bumper;

                if (liftUp && !liftDown) {
                    r2.liftMotor(r2.UP);
                }
                else if (!liftUp && liftDown) {
                    r2.liftMotor(r2.DOWN);
                }
                else {
                    r2.liftMotorStop();
                }

                telemetry.addData("LiftUp", liftUp);
                telemetry.addData("LiftDown", liftDown);
                telemetry.update();
            }
        }
    }
}
