package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.Color;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.EmergencyStopException;
import org.firstinspires.ftc.teamcode.PowerPlay.Utilities.iRobot;

public class PowerPlayRobot implements iRobot {
    private final LinearOpMode creator;
    private final HardwareMap hardwareMap;
    public Telemetry telemetry;

    private DcMotorEx rfMotor;
    private DcMotorEx rrMotor;
    private DcMotorEx lfMotor;
    private DcMotorEx lrMotor;

    private CRServo liftServo;
    private CRServo intakeServo;

    private AnalogInput potentiometer;
    private BNO055IMU imu;
    private DigitalChannel greenLED;
    private DigitalChannel redLED;

    public final DcMotorSimple.Direction UP = DcMotorSimple.Direction.REVERSE;
    public final DcMotorSimple.Direction DOWN = DcMotorSimple.Direction.FORWARD;

    private final double MAX_ROBOT_SPEED = 0.80; // The maximum speed we want our robot to drive at.
    @SuppressWarnings("FieldCanBeLocal")
    private final double MIN_ROBOT_SPEED = 0.40; // The minimum speed we can have our robot to drive at.
    @SuppressWarnings("FieldCanBeLocal")
    private final double correctionSpeed = 0.1;

    private final double wheelCircumferenceInInches = (96 / 25.4) * Math.PI;
    // TODO: PIDF values must be updated to work for this year.
    // Suppression is due to lack of strafe code.
    @SuppressWarnings("FieldCanBeLocal")
    private final int lfMotorMaxTps = 2655;
    @SuppressWarnings("FieldCanBeLocal")
    private final int rfMotorMaxTps = 2650;
    @SuppressWarnings("FieldCanBeLocal")
    private final int lrMotorMaxTps = 2610;
    @SuppressWarnings("FieldCanBeLocal")
    private final int rrMotorMaxTps = 2615;

    private final double ticksPerMotorRevolution = 530.3;
    private final double ticksPerInch = ticksPerMotorRevolution / wheelCircumferenceInInches;

    // TODO: PIDF values must be updated to work for this year.
    private final double drivePositionPIDF1 = 2.0; // For distance >= 20"
    private final double drivePositionPIDF2 = 4.0; // For distances <= 20"

    private double delta;
    private final double deltaThreshold = 1;

    public PowerPlayRobot(LinearOpMode creator) {
        this.creator = creator;
        this.hardwareMap = creator.hardwareMap;
        this.telemetry = creator.telemetry;
    }
    @Override
    public void initHardware() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");

        liftServo = hardwareMap.get(CRServo.class, "LiftServo");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");

        potentiometer = hardwareMap.get(AnalogInput.class, "LiftAngleSensor");
        greenLED = hardwareMap.get(DigitalChannel.class, "GreenLED");
        redLED = hardwareMap.get(DigitalChannel.class, "RedLED");

        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeIMU();
    }

    /**
     * Initializes IMU to give us the robots heading
     */
    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        telemetry.addData("Status", "Calibrating IMU...done");
        telemetry.update();
    }

    public double getPotentiometer() {
        double voltage = potentiometer.getVoltage();
        telemetry.addData("Potentiometer", voltage);
        return voltage;
    }

    /**
     * Displays telemetry information on the Driver Hub
     */
    public void telemetryDashboard(@SuppressWarnings("unused") String msg) {
        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                getIMUHeading(), getIMUHeading(), delta);

        telemetry.addData("Target", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getTargetPosition(), lrMotor.getTargetPosition(), rfMotor.getTargetPosition(), rrMotor.getTargetPosition());
        telemetry.addData("Position", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getCurrentPosition(), lrMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rrMotor.getCurrentPosition());
        telemetry.addData("Power", "LF: %.1f, LR: %.1f, RF: %.1f, RR: %.1f",
                lfMotor.getPower(), lrMotor.getPower(), rfMotor.getPower(), rrMotor.getPower());

        double imuHeading = getIMUHeading();

        telemetry.addData("IMU Heading", "%.0f", imuHeading);
        telemetry.update();
    }

    /**
     * @param direction  1 = forward, 0 = stop, -1 = backwards
     * @param accelSlope The acceleration slope
     * @param decelSlope The deceleration slope
     * @return Returns true if drive should exit, false if it may continue
     */
    public boolean driveAsserts(int direction, double accelSlope, double decelSlope) {
        // We are checking to make sure it is doing what we think it should be
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

    private void driveDelta(double desiredHeading, double power) {
        double currentHeading = getIMUHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        double adjustSpeed = 0;


        if (Math.abs(delta) > deltaThreshold) {
            adjustSpeed = correctionSpeed;
            if (delta > 0) {
                adjustSpeed *= -1;
            }
        }

        double leftSpeed = power + adjustSpeed;
        double rightSpeed = power - adjustSpeed;

        if (leftSpeed > MAX_ROBOT_SPEED) {
            leftSpeed = MAX_ROBOT_SPEED;
        }
        if (rightSpeed > MAX_ROBOT_SPEED) {
            rightSpeed = MAX_ROBOT_SPEED;
        }
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        System.out.println("DEBUG: Delta power (left): " + leftSpeed + " (right): " + rightSpeed);
    }

    public void liftMotor(DcMotorSimple.Direction direction) {
        liftServo.setDirection(direction);
        liftServo.setPower(1.00);
        telemetry.addData("LiftPower", liftServo.getPower());
    }

    public void liftMotorStop() {
        liftServo.setPower(0);
    }

    public void intakeMotor(DcMotorSimple.Direction direction) {
        intakeServo.setDirection(direction);
        intakeServo.setPower(0.60);
        telemetry.addData("IntakePower", intakeServo.getPower());
    }

    public void intakeStop() {intakeServo.setPower(0);}

    /**
     * @param distance Distance the robot should travel in inches, positive for forwards, negative for backwards
     */
    @Override
    public void drive (double distance) {
        drive(distance, MIN_ROBOT_SPEED, MAX_ROBOT_SPEED);
    }

    /**
     *
     * @param distance Distance the robot should travel in inches, positive for forwards, negative for backwards
     * @param minSpeed Minimum speed the robot should drive at ALWAYS POSITIVE, Range: 0-1
     * @param maxSpeed Maximum speed the robot should drive at ALWAYS POSITIVE, Range: 0-1
     */
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
        }
        else {
            minPower = minSpeed;
        }

        double maxPower;
        if (direction == -1) {
            maxPower = -1 * (maxSpeed);
        }
        else {
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
    public void driveTank(double l, double r, double b) {
        // The normal speed the robot should travel at
        double normalSpeed = 0.50;

        lfMotor.setPower(l * normalSpeed);
        rfMotor.setPower(r * normalSpeed);
        lrMotor.setPower(l * normalSpeed);
        rrMotor.setPower(r * normalSpeed);
    }

    public void setLEDColor(Color color) {
        if (color == Color.GREEN) {
            redLED.setState(false);
            greenLED.setState(true);
        }
        else if (color == Color.RED) {
            redLED.setState(true);
            greenLED.setState(false);
        }
        else if (color == Color.YELLOW) {
            redLED.setState(true);
            greenLED.setState(true);
        }
    }

    @Override
    public void driveXYRB(double x, double y, double r, double b) {
        // The normal speed our robot should be driving at.
        double normalSpeed = 0.65;

        /*
            Because we use Mecanum wheels, we can move forward, rotate, and strafe.
            Here, we are taking into account the direction each wheel should travel at in
            order to move in the direction we want the robot to move.
        */
        // Left Front motor speed.
        double lfSpeed = -((y - x - r) * normalSpeed);
        // Right Front motor speed.
        double rfSpeed = -((y + x + r) * normalSpeed);
        // Left Rear motor speed.
        double lrSpeed = -((y + x - r) * normalSpeed);
        // Right Rear motor speed.
        double rrSpeed = -((y - x + r) * normalSpeed);

        // The acceleration speed set on normal speed.
        double accelerationSpeed = MAX_ROBOT_SPEED - normalSpeed;
        if (Math.abs(lfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lfSpeed) > normalSpeed) {
                if (lfSpeed > 0) {
                    lfSpeed = normalSpeed;
                    lfMotor.setPower(lfSpeed + accelerationSpeed * b);
                } else if (lfSpeed < 0) {
                    lfSpeed = -normalSpeed;
                    lfMotor.setPower(lfSpeed - accelerationSpeed * b);
                } else {
                    lfMotor.setPower(0);
                }
            }
        }
        else {
            if (lfSpeed > 0) {
                lfMotor.setPower(lfSpeed + accelerationSpeed * b);
            } else if (lfSpeed < 0) {
                lfMotor.setPower(lfSpeed - accelerationSpeed * b);
            } else {
                lfMotor.setPower(0);
            }
        }

        if (Math.abs(rfSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rfSpeed) > normalSpeed) {
                if (rfSpeed > 0) {
                    rfSpeed = normalSpeed;
                    rfMotor.setPower(rfSpeed + accelerationSpeed * b);
                } else if (rfSpeed < 0) {
                    rfSpeed = -normalSpeed;
                    rfMotor.setPower(rfSpeed - accelerationSpeed * b);
                } else {
                    rfMotor.setPower(0);
                }
            }
        }
        else {
            if (rfSpeed > 0) {
                rfMotor.setPower(rfSpeed + accelerationSpeed * b);
            } else if (rfSpeed < 0) {
                rfMotor.setPower(rfSpeed - accelerationSpeed * b);
            } else {
                rfMotor.setPower(0);
            }
        }

        if (Math.abs(lrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(lrSpeed) > normalSpeed) {
                if (lrSpeed > 0) {
                    lrSpeed = normalSpeed;
                    lrMotor.setPower(lrSpeed + accelerationSpeed * b);
                } else if (lrSpeed < 0) {
                    lrSpeed = -normalSpeed;
                    lrMotor.setPower(lrSpeed - accelerationSpeed * b);
                } else {
                    lrMotor.setPower(0);
                }
            }
        }
        else {
            if (lrSpeed > 0) {
                lrMotor.setPower(lrSpeed + accelerationSpeed * b);
            } else if (lrSpeed < 0) {
                lrMotor.setPower(lrSpeed - accelerationSpeed * b);
            } else {
                lrMotor.setPower(0);
            }
        }

        if (Math.abs(rrSpeed) + accelerationSpeed * b > MAX_ROBOT_SPEED) {
            if (Math.abs(rrSpeed) > normalSpeed) {
                if (rrSpeed > 0) {
                    rrSpeed = normalSpeed;
                    rrMotor.setPower(rrSpeed + accelerationSpeed * b);
                } else if (rrSpeed < 0) {
                    rrSpeed = -normalSpeed;
                    rrMotor.setPower(rrSpeed - accelerationSpeed * b);
                } else {
                    rrMotor.setPower(0);
                }
            }
        }
        else {
            if (rrSpeed > 0) {
                rrMotor.setPower(rrSpeed + accelerationSpeed * b);
            } else if (rrSpeed < 0) {
                rrMotor.setPower(rrSpeed - accelerationSpeed * b);
            } else {
                rrMotor.setPower(0);
            }
        }
        telemetry.addData("LF", lfMotor.getPower());
        telemetry.addData("LR", lrMotor.getPower());
        telemetry.addData("RF", rfMotor.getPower());
        telemetry.addData("RR", rrMotor.getPower());
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

    /**
     * Powers all 4 of the robot's wheels.
     * If the motor mode is set to RUN_USING_ENCODER, then PTW sets the velocity.
     * If the motor mode is set to RUN_TO_POSITION, then PTW sets the power.
     * <p>
     * The robot is only capable of accepting speeds of -1 --> 1.
     * If you give a value out of that range, PTW will scale down the numbers appropriately.
     *
     * @param lfPower power/velocity applied to the left front wheel.
     * @param lrPower power/velocity applied to the left rear wheel.
     * @param rfPower power/velocity applied to the right front wheel.
     * @param rrPower power/velocity applied to the right rear wheel.
     */
    private void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower) {
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max(leftMax, rightMax);

        if (max > MAX_ROBOT_SPEED) {
            lfPower /= max;
            lrPower /= max;
            rfPower /= max;
            rrPower /= max;
        }

        if (lfMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
            double lfVelocity = lfPower * lfMotorMaxTps;
            double lrVelocity = lrPower * lrMotorMaxTps;
            double rfVelocity = rfPower * rfMotorMaxTps;
            double rrVelocity = rrPower * rrMotorMaxTps;

            if (creator.opModeIsActive()) {

                lfMotor.setVelocity(lfVelocity);
                lrMotor.setVelocity(lrVelocity);
                rfMotor.setVelocity(rfVelocity);
                rrMotor.setVelocity(rrVelocity);
            } else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        } else {
            // We assume that we will be using RUN_TO_POSITION mode.
            if (creator.opModeIsActive()) {
                lfMotor.setPower(lfPower);
                lrMotor.setPower(lrPower);
                rfMotor.setPower(rfPower);
                rrMotor.setPower(rrPower);
            } else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
    }

    @Override
    public double getIMUHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * @return returns the distance the robot has traveled in inches
     */
    private double getMotorPosition() {
        double lfPosition = lfMotor.getCurrentPosition();
        double rfPosition = rfMotor.getCurrentPosition();
        double lrPosition = lrMotor.getCurrentPosition();
        double rrPosition = rrMotor.getCurrentPosition();

        double motorPositionAverage = (lfPosition + rfPosition + lrPosition + rrPosition) / 4;

        return motorPositionAverage / ticksPerInch;
    }

    private double powerPercentage(double delta) {
        double powerPercent = -0.000027 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }

    @Override
    public double normalizeHeading(double heading) {
        return 0;
    }
}
