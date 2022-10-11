package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The is an interface (specifies behavior other classes must implement).
 *
 */
public interface iRobot {
    /**
     * initHardware maps and initializes necessary hardware at the beginning of the match.
     */
    void initHardware();

    /**
     * drive is a Autonomous method that moves the robot forwards or backwards ONLY.
     * All inputs are relative to the front of the robot.
     *
     * @param distance Accepts a positive or negative number representing the number of inches to move
     *                 Negative is backwards, and positive is forwards
     */
    void drive(double distance);

    /**
     * drive is a Autonomous method that moves the robot right or left ONLY.
     * All inputs are relative to the front of the robot (looking at it from behind, as the robot)
     *
     * @param distance Accepts a positive or negative number representing the number of inches to move.
     *                 Positive is left, and negative is right
     */
    void strafe(double distance);

    /**
     * rotate is a Autonomous method that rotates the robot.
     * All inputs are relative to the direction the robot was facing on initialization (in degrees)
     *
     * @param degrees Accepts between -180 and 180. Negative degrees is clockwise, and positive degrees is counterclockwise.
     */
    void rotate(double degrees);

    /**
     * driveXYRB is a TeleOp method that moves the robot on the x axis the y axis and rotates.
     * All inputs are relative to the front of the robot.
     *  @param x Accepts between -1.0 and 1.0 Negative x is left, and positive x is right
     * @param y Accepts between -1.0 and 1.0 Negative y is backwards, and positive y is forwards
     * @param r (rotate)Accepts between -1.0 and 1.0 Negative degrees is counterclockwise, and positive degrees is clockwise.
     * @param b (boost) Accepts between 0 and 1.0. The percentage of extra speed that you want the robot to go past the normalSpeed.
     */
    void driveXYRB(double x, double y, double r, double b);

    /**
     * Gets the IMU Heading for drive
     */
    double getIMUHeading();
    /**
     * driveStop is a TeleOp method that stops the robot driving by stopping the wheels
     */
    void driveStop();

    /**
     * TODO: JavaDoc
     * @param mode Regular DC motor modes
     */
    void setMotorMode(DcMotor.RunMode mode);

    /**
     * stopAll is a TeleOp method that stops all functions of the robot by powering down all motors
     */
    void stopAll();

    /**
     * normalizeHeading is a TeleOp method that
     */
    double normalizeHeading(double heading);
}
