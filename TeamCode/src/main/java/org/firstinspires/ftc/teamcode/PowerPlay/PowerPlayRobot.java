package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.iRobot;

public class PowerPlayRobot implements iRobot {
    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;
    private BNO055IMU imu;

    private final double MAX_ROBOT_SPEED = 0.80; // The maximum speed we want our robot to drive at.
    private final double MIN_ROBOT_SPEED = 0.40; // The minimum speed we can have our robot to drive at.

    private final double wheelCircumferenceInInches = (96 / 25.4) * Math.PI;
    private final double ticksPerMotorRevolution = 530.3;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;

    private final double drivePositionPIDF1 = 2.0; // For distance >= 20"
    private final double drivePositionPIDF2 = 4.0; // For distances <= 20"

    @Override
    public void initHardware() {

    }

    /**
     * @param direction  1 = forward, 0 = stop, -1 = backwards
     * @param accelSlope The acceleration slope
     * @param decelSlope The deceleration slope
     * @return Returns true if drive should exit, false if it may continue
     */
    public boolean driveAsserts(int direction, double accelSlope, double decelSlope) {
        //we are checking to make sure it is doing what we think it should be
        if (direction == -1 && accelSlope > 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate forwards when you said" +
                    "to go backwards. ");
            return true;
        }
        if (direction == 1 && accelSlope < 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate backwards when you said" +
                    "to go forward. ");
            return true;
        }
        if (direction == 0 && accelSlope != 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to accelerate when you said" +
                    "to go nowhere. ");
            return true;
        }
        if (direction == -1 && decelSlope < 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate backwards when you said" +
                    "to go backwards. ");
            return true;
        }
        if (direction == 1 && decelSlope > 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate forwards when you said" +
                    "to go forwards. ");
            return true;
        }
        if (direction == 0 && decelSlope != 0) {
            System.out.println("ERROR: Uh-oh, the robot tried to decelerate  when you said" +
                    "to go nowhere. ");
            return true;
        } else {
            return false;
        }
    }
    @Override
    public void drive (double distance) {
        drive(distance, MIN_ROBOT_SPEED, MAX_ROBOT_SPEED);
    }

    public void drive(double distance, double minSpeed, double maxSpeed) {

        if(distance >= 20) {
            lfMotor.setPositionPIDFCoefficients(drivePositionPIDF1);
            rfMotor.setPositionPIDFCoefficients(drivePositionPIDF1);
            lrMotor.setPositionPIDFCoefficients(drivePositionPIDF1);
            rrMotor.setPositionPIDFCoefficients(drivePositionPIDF1);
        }

        double desiredHeading = getIMUHeading();
        if (distance == 0) {
            System.out.println("Success! The robot did not move. The distance entered was 0.");
            return;
        }

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

        int direction = (distance > 0) ? 1 : -1;
        System.out.println("Direction: " + direction);

        double power;

        double minPower;
        if (direction == -1) {
            minPower = -1 * (minSpeed);
        } else {
            minPower = minSpeed;
        }

        double maxPower;
        if (direction == -1) {
            maxPower = -1 * (maxSpeed);
        } else {
            maxPower = maxSpeed;
        }

        //rise-over-run code for accel/decel slope
        double accelRun = 2; //inches to accelerate up to max speed.
        double decelRun = 4; //inches to decelerate down to stopping.
        System.out.println("DEBUG: Distance = " + accelRun + decelRun);

        double accelRise;
        if (direction == -1) {
            accelRise = -1 * (maxSpeed - minSpeed);
        } else {
            accelRise = maxSpeed - minSpeed;
        }

        double decelRise;
        if (direction == -1) {
            decelRise = -1 * (0 - maxSpeed);
        } else {
            decelRise = 0 - maxSpeed;
        }

        double accelSlope = accelRise / accelRun;

        //decel goes from max to stopped
        double decelSlope = decelRise / decelRun;

        if (driveAsserts(direction, accelSlope, decelSlope)) {
            return;
        }

        
        telemetryDashboard("");
        double distanceTraveled = getMotorPosition();

        if (Math.abs(distance) <= accelRun + decelRun) {
            System.out.println("DEBUG: Going less than 6 inches");
            //Cruising speed
            while (Math.abs(distance) > Math.abs(distanceTraveled)) {
                distanceTraveled = getMotorPosition();
                power = maxPower;
                System.out.println("DEBUG: " + power);
                powerTheWheels(power, power, power, power);
            }
        }
        else {
            while ((Math.abs(distance)) > (Math.abs(distanceTraveled))) {
                //Acceleration to cruising speed
                System.out.println("DEBUG Accelerating: ");
                while ((Math.abs(accelRun)) > (Math.abs(distanceTraveled))) {
                    distanceTraveled = getMotorPosition();
                    power = Math.abs(distanceTraveled) * accelSlope + minPower;
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }

                //Cruising speed
                System.out.println("DEBUG Cruising: ");
                while (((Math.abs(distance)) - (Math.abs(distanceTraveled))) > decelRun) {
                    distanceTraveled = getMotorPosition();
                    power = maxPower;
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }

                distanceTraveled = getMotorPosition();
                distance -= distanceTraveled;
                setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

                //Deceleration to stopping
                System.out.println("DEBUG Decelerating: ");
                while ((Math.abs(distance)) > (Math.abs(distanceTraveled))) {
                    distanceTraveled = getMotorPosition();
                    power = maxPower + distanceTraveled * decelSlope;
                    if(power <= minPower) {
                        power = minPower;
                    }
                    driveDelta(desiredHeading, power);
                    System.out.println("DEBUG Speed: " + power + ", Distance Travelled: " + distanceTraveled + ", Desired Distance: " + distance);
                }
                System.out.println("DEBUG: Finished decel");
            }
        }
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }


    @Override
    public void strafe(double distance) {

    }

    @Override
    public void rotate(double degrees) {

    }

    @Override
    public void driveXYRB(double x, double y, double r, double b) {

    }

    @Override
    public void driveStop() {

    }

    @Override
    public void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    @Override
    public void stopAll() {

    }
    /**
     * Programs all four motors to run to position, based off of distance and direction.
     *
     * @param distance  The distance you want to drive in inches
     * @param direction The direction each motor should turn. It is an array consisting of the
     *                  LfMotor, LrMotor, RfMotor, and RrMotor. The values can be -1 to move backwards,
     *                  1 to move forwards, or 0 to not move the motor at all.
     */
    private void setMotorDistanceToTravel(double distance, int[] direction) {

        if (direction.length != 4) {
            throw new IllegalArgumentException("You must provide an array with exactly 4 elements!");
        }

        for (int i = 0; i < 4; i++) {
            if (direction[i] > 1 || direction[i] < -1) {
                throw new IllegalArgumentException("Elements must be -1, 0, or 1.");
            }
        }

        double distanceInTicks = distance * ticksPerInch;
        int leftFrontTargetPosition = (int) (lfMotor.getCurrentPosition() + distanceInTicks);
        int leftRearTargetPosition = (int) (lrMotor.getCurrentPosition() + distanceInTicks);
        int rightFrontTargetPosition = (int) (rfMotor.getCurrentPosition() + distanceInTicks);
        int rightRearTargetPosition = (int) (rrMotor.getCurrentPosition() + distanceInTicks);

        lfMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        lrMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        rfMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        rrMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public double getIMUHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    @Override
    public double normalizeHeading(double heading) {
        return 0;
    }
}
