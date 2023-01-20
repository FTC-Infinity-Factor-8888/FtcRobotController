package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.Color;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.IntakePosition;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.LED;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.WristPosition;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        boolean liftUp;
        boolean liftDown;

        boolean intake;
        boolean outtake;

        boolean wristUp;
        boolean wristDown;
        boolean wristMiddle;

        boolean lb;
        boolean rb;
        boolean y;
        boolean a;

        double forwardInput;
        double strafeInput;
        double rotateInput;
        double accelerator;
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

                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.

                // Controls to allow our robot to reach speeds up to maxSpeed.
                accelerator = gamepad1.right_trigger;

                r2.driveXYRB(strafeInput, forwardInput, rotateInput, accelerator);

                liftUp = gamepad2.right_bumper;
                liftDown = gamepad2.left_bumper;

                if (liftUp && !liftDown) {
                    r2.liftMotor(0.50);
                }
                else if (!liftUp && liftDown) {
                    r2.liftMotor(-0.20);
                }
                else {
                    r2.liftMotorStop();
                }

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

                /* Here we show values on the driver hub that may be useful to know while driving
                the robot or during testing. */
                r2.telemetryDashboard("");
                telemetry.update();
            }
        }
    }
}
