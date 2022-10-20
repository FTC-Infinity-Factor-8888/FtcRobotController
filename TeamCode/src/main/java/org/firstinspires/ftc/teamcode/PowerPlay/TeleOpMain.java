package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    PowerPlayRobot r2;

     //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here.

        // Declaring the buttons that may quickly change.
        boolean currentLiftUp;
        boolean currentLiftDown;

        double accelerator;
        double duckWheelAccelerator;

        // Declaring the former values of the buttons, so we can tell if they changed.
        boolean priorLiftUp = false;
        boolean priorLiftDown = false;

        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initHardware();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double forwardInput = gamepad1.left_stick_y; // Controls for moving back and forward.
                double strafeInput = gamepad1.left_stick_x; // Controls for strafing.
                double rotateInput = gamepad1.right_stick_x; // Controls for pivoting.

                // Controls to allow our robot to reach speeds up to maxSpeed.
                accelerator = gamepad1.right_trigger;
                duckWheelAccelerator = gamepad2.right_trigger;

                currentLiftUp = gamepad2.right_bumper; // Controls for moving the vertical-lift up.
                currentLiftDown = gamepad2.left_bumper; // Controls for moving the lift down.

                /*
                Lift requirements:
                    The first button pressed sets the direction.
                    Holding both buttons stops the motion.
                    Releasing one of those two buttons heads in the direction of the remaining button.
                    Releasing both buttons stops the motion.

                Lift requirements (video game option, not chosen):
                    The first button to be pressed sets the direction that we are going.
                    The other button is ignored while the first button is being held down.
                    Both buttons must be released to stop the lift
                    or if it reaches its target position.
                    The lift must be stopped in order to switch direction.

                DuckWheel requirements:
                    The button that moves
                 */

                // To control the lift.
                // Checking to see whether the buttons are still pressed.
                if(currentLiftUp != priorLiftUp || currentLiftDown != priorLiftDown) {
                    // 0 = no motion, 1 = up, -1 = down
                    // Checking to see which buttons are pushed.
                    int direction = (currentLiftUp?1:0) + (currentLiftDown?-1:0);
                    if(liftOverride) {
                        r2.liftMotorOverride(direction);
                    }
                    else {
                        r2.liftMotor(direction);
                    }
                    telemetry.addData("Lift:", r2.getCurrentLiftPosition());
                }

                if(liftOverride != priorLiftOverride && !liftOverride) {
                    r2.liftMotorReset(); // Resets the liftMotor's encoder
                }




                r2.driveXYRB(strafeInput, forwardInput, rotateInput, accelerator);

                /* Here we show values on the driver hub that may be useful to know while driving
                the robot or during testing. */
                telemetry.addData("Lift", r2.getCurrentLiftPosition());
                telemetry.addData("Accelerator", accelerator);
                telemetry.update();

                /* Here we set the current button positions to the prior button position, so we have
                updated data as we loop back.

                Note: This loops multiple times per millisecond.
                 */
                priorLiftUp = currentLiftUp;
                priorLiftDown = currentLiftDown;

            }
        }
    }
}
