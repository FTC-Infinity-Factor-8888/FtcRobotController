package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.Color;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.IntakePosition;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.LED;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.WristPosition;

@TeleOp(name = "JoystickXYRBDrive")
public class JoystickXYRBDrive extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        double forwardInput;
        double strafeInput;
        double rotateInput;
        double direction = 1.0;

        r2 = new PowerPlayRobot(this);
        r2.initHardware();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.

                r2.teleOpTestDrive(strafeInput, forwardInput, rotateInput);
            }
        }
    }
}
