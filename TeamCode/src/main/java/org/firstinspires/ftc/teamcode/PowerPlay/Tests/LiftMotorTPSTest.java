package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.LiftDirection;

@TeleOp(name = "LiftMotorTPSTest")
public class LiftMotorTPSTest extends LinearOpMode {
    PowerPlayRobot r2;
    double sum = 0.0;
    double index = 0.0;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        boolean liftUp;
        boolean liftDown;
        // Declaring the former values of the buttons, so we can tell if they changed.
        // Note: Currently no such variables

        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initHardware();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                liftUp = gamepad2.left_bumper;
                liftDown = gamepad2.right_bumper;
                if (liftUp && !liftDown) {
                    r2.liftMotor(LiftDirection.UP, 860);
                }
                else if (!liftUp && liftDown) {
                    r2.liftMotor(LiftDirection.DOWN, 100);
                }
                else {
                    r2.liftMotorStop();
                }
                telemetry.addData("Power: ", r2.liftMotor.getPower());
                telemetry.addData("Position: ", r2.liftMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
