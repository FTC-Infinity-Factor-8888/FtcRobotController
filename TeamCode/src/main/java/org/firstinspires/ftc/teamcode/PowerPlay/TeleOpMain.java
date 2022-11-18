package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        double forwardInput;
        double strafeInput;
        double rotateInput;
        double accelerator;

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

                forwardInput = gamepad1.left_stick_y; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.

                // Controls to allow our robot to reach speeds up to maxSpeed.
                accelerator = gamepad1.right_trigger;

                r2.driveXYRB(strafeInput, forwardInput, rotateInput, accelerator);

                /*
                TODO: Reenable the code after the motors have been added.
                liftUp = gamepad2.right_bumper;
                liftDown = gamepad2.left_bumper;

                if (liftUp && !liftDown) {
                    r2.liftMotor(r2.UP);
                }
                else if (!liftUp && liftDown) {
                    r2.liftMotor(r2.DOWN);
                }
                else {
                    r2.liftMotorStop();
                }
                */

                intake = gamepad2.a;
                outtake = gamepad2.y;
                if (intake && !outtake) {
                    r2.intakeMotor(DcMotorSimple.Direction.FORWARD);
                }
                else if (!intake && outtake) {
                    r2.intakeMotor(DcMotorSimple.Direction.REVERSE);
                }
                else {
                    r2.intakeStop();
                }

                /* Here we show values on the driver hub that may be useful to know while driving
                the robot or during testing. */
                telemetry.addData("Accelerator", accelerator);
                telemetry.addData("Y", gamepad2.y);
                telemetry.addData("A", gamepad2.a);
                r2.getPotentiometer();
                telemetry.update();
            }
        }
    }
}
