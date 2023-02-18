package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.Color;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.IntakePosition;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.LED;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.WristPosition;

@TeleOp(name = "TeleOpBeta")
public class TeleOpBeta extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        boolean liftUp;
        boolean liftDown;
        boolean liftOverride;
        double liftDecelerator;

        boolean intake;
        boolean outtake;

        boolean wristUp;
        boolean wristDown;
        boolean wristMiddle;
        double wristPosition;
        boolean setWristPosition;

        boolean lb;
        boolean rb;
        boolean y;
        boolean a;

        double forwardInput;
        double strafeInput;
        double rotateInput;
        double accelerator; // A magnitude of acceleration
        double decelerator; // A magnitude of deceleration

        double liftPower;

        int direction = 1;

        // Declaring the former values of the buttons, so we can tell if they changed.
        // Note: Currently no such variables

        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initHardware();
        telemetry.addData("Version", r2.teleOpVersion);
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                lb = gamepad1.left_bumper;
                rb = gamepad1.right_bumper;
                y = gamepad1.y;
                a = gamepad1.a;

                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.
                accelerator = gamepad1.right_trigger;
                decelerator = gamepad1.left_trigger;

                liftUp = gamepad2.right_bumper;
                liftDown = gamepad2.left_bumper;
                liftOverride = gamepad2.b;
                liftDecelerator = gamepad2.left_trigger;

                intake = gamepad2.a;
                outtake = gamepad2.y;

                wristUp = gamepad2.dpad_up;
                wristDown = gamepad2.dpad_down;
                wristMiddle = gamepad2.dpad_right;
                wristPosition = -gamepad2.left_stick_y;  // The joysticks are inverted to make up the positive direction and down the negative direction
                setWristPosition = gamepad2.x;

                liftPower = -gamepad2.right_stick_y;

                // Drive inversion code
                if (lb && rb) {
                    if (y) {
                        direction = 1;
                        r2.setLEDColor(LED.FRONT, Color.GREEN);
                        r2.setLEDColor(LED.REAR, Color.RED);
                    }
                    else if (a) {
                        direction = -1;
                        r2.setLEDColor(LED.FRONT, Color.RED);
                        r2.setLEDColor(LED.REAR, Color.GREEN);
                    }
                }

                // Drive code
                if (accelerator != 0 && decelerator == 0) {  // Accelerate
                    r2.driveXYRB(strafeInput, forwardInput, rotateInput, accelerator, 1.0);
                    telemetry.addData("Acceleration","accelerating");
                }
                else if (accelerator == 0 && decelerator != 0) {  // Decelerate
                    r2.driveXYRB(strafeInput, forwardInput, rotateInput, decelerator, -1.0);
                    telemetry.addData("Acceleration", "decelerating");
                }
                else {  // If both triggers are pressed, don't accelerate in either direction
                    r2.driveXYRB(strafeInput, forwardInput, rotateInput, 0.0, 1.0);
                    telemetry.addData("Acceleration", "null");
                }

                // Lift code
                if (liftUp && !liftDown) {
                    r2.liftMotor(0.50 - 0.50 * liftDecelerator);
                }
                else if (!liftUp && liftDown) {
                    r2.liftMotor(-0.20);
                }
                else {
                    r2.liftMotorHold();
                }

                if (liftOverride) {
                    r2.liftMotorStop();
                }

                r2.liftMotor(liftPower);

                // Intake code
                if (intake && !outtake) {
                    r2.intakeMotor(IntakePosition.IN);
                }
                else if (!intake && outtake) {
                    r2.intakeMotor(IntakePosition.OUT);
                }

                // Wrist presets code
                if (r2.getPotentiometer() <= 1.195 && r2.getPotentiometer() >=  0.934) {
                    r2.wristMotor(WristPosition.MIDDLE);
                }
                else if (wristUp && !wristDown && !wristMiddle) {
                    r2.wristMotor(WristPosition.UP);
                }
                else if (!wristUp && wristDown && !wristMiddle) {
                    r2.wristMotor(WristPosition.DOWN);
                }
                else if (!wristUp && !wristDown && wristMiddle) {
                    r2.wristMotor(WristPosition.MIDDLE);
                }

                // Wrist manual code
                if (setWristPosition) {
                    r2.wristMotor(wristPosition);
                }

                // Only handles liftPower
                r2.endOfLoop();

                /* Here we show values on the driver hub that may be useful to know while driving
                the robot or during testing. */
                r2.telemetryDashboard("");
                telemetry.update();
            }
        }
    }
}
