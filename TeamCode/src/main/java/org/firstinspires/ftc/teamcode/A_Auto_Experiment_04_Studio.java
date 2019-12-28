package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;

@Autonomous(name = "A_Auto_Experiment_04 (BTJ-Studio)", group = "")
public class A_Auto_Experiment_04_Studio extends LinearOpMode {

    private DcMotor motor0;
    private DcMotor motor2;
    private DcMotor motor1;
    private DcMotor motor3;
    private Servo servo1;
    private Servo servo2;
    private BNO055IMU imu;

    private int flagWhiskers = 0;
    //private int flagGrabber = 0;
    private int directionModifier = 1;
    private int parkPosition = 1;
    //private int skystonePattern = 0;
    //private double skyStoneY = 0;
    private int allianceSide = 1;
    private int robotType = 1;

    private double minTurnSpeed = 0.06;
    private double demoDriveSpeed = 1;
    private double demoStrafeSpeed = 1;
    private double demoTurnSpeed = 0.5;
    private double pulsesPerInch = (383.6 / (3.933 * Math.PI)) * 2;
    private double strafeDistanceFactor = 1.073;

    //private ElapsedTime skystoneTimer;
    private ElapsedTime elapsedTime;

    //private static final String VUFORIA_KEY = "AWTeEcT/////AAABmVz1P4UdL0MNjgGWLq6XTd0rW7UNlrpH0vTKgfinUVyzoLMjbYnsVIGWhRT4FPZGkhAUkRXUl7fkApwNdoaFtn2T32Yo/dl47pZhNG0vFeohuNE4O6aoogDSrfhpo/nBjoyCEqyLoxiwYixxCfj2L8Vn5F4qFEw4ezFTjhAfvlBljbVJkYqHakN+v2AxiTxMIAAASLMDJDR4pEfd4E8y3mApNEbnDc73+YLq59fEaQhgkQRrWy+nQ14kPxD0R7afBY7vT6vN6XWC7I7UBo7J4Z7EGDksSlSrUVxeNaW7TUquOr1FEj6F3Jnep/RhJarAo+Ts6yVsK6eka5486Be9O9qxAdxmyYh2WXOVNmsQXgAy";
    //private VuforiaBase.TrackingResults vuforiaResults;


    /**
     * Describe this function...
     */
    private void AutoThing() {
        flagWhiskers = 0;
        stowWhiskers();
        // Drive toward foundation
        goDrive(0.5, 24, 0 * directionModifier, 1);
        // Strafe to middle of foundation
        goStrafe(0.25, 12 * directionModifier, 0 * directionModifier, 1);
        // Drive into foundation and grab it
        flagWhiskers = 6;
        goDrive(0.15, 12, 0 * directionModifier, 1);
        // Back up a little at a shallow angle
        goDrive(0.5, -12, 13 * directionModifier, 1);
        // Back up more at larger angle
        goDrive(0.5, -24, 30 * directionModifier, 1);
        // Push foundation square to side
        goDrive(0.5, 18, 90 * directionModifier, 1);
        // Open whiskers and wait for them to clear
        stowWhiskers();
        sleep(500);
        // Back away from foundation
        goDrive(0.5, -12, 90 * directionModifier, 1);
        // Strafe into parking position
        // 1 run into wall
        goStrafe(0.25, 24 * directionModifier, 90 * directionModifier, 1);
        // 2 move to far position if necessary
        if (parkPosition == 2) {
            goStrafe(0.25, -26 * directionModifier, 90 * directionModifier, 1);
        }
        // Back under Skybridge
        goDrive(0.5, -30, 90 * directionModifier, 1);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor3 = hardwareMap.dcMotor.get("motor3");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            AutoThing();
            while (opModeIsActive()) {
                // Display orientation info.
                telemetry.addData(">", "Running");
                telemetry.addData("Heading (Rot about Z)", getHeading());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void initialize() {
        setConstantsAndVars();
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Set motor directions and modes
        initMotors();
        initIMU();
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Heading (Rot about Z)", getHeading());
            telemetry.addData("-", "------------");
            telemetry.addData("Alliance", allianceSide == 1 ? "BLUE" : (allianceSide == 2 ? "RED" : "!!!!  problem !!!!"));
            telemetry.addData("Park Position", parkPosition == 1 ? "NEAR" : (parkPosition == 2 ? "FAR" : "!!!!  problem !!!!"));
            telemetry.update();
            if (gamepad1.x) {
                allianceSide = 1;
                directionModifier = 1;
            }
            if (gamepad1.b) {
                allianceSide = 2;
                directionModifier = -1;
            }
            if (gamepad1.a) {
                parkPosition = 1;
            }
            if (gamepad1.y) {
                parkPosition = 2;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void setConstantsAndVars() {
        // 1 = Blue, 2 = Red
        allianceSide = 1;
        // 1 = ParkNear, 2 = ParkFar
        parkPosition = 1;
        // 1 = Dog, 2 = GoBilda
        robotType = 1;
        // -------------
        if (allianceSide == 1) {
            // Blue
            directionModifier = 1;
        } else if (allianceSide == 2) {
            // Red
            directionModifier = -1;
        }
        if (parkPosition == 1) {
            // Near
        } else if (parkPosition == 2) {
            // Far
        }
        if (robotType == 2) {
            // Gobilda Surrogate
            minTurnSpeed = 0.06;
            demoDriveSpeed = 1;
            demoStrafeSpeed = 1;
            demoTurnSpeed = 0.5;
            pulsesPerInch = (383.6 / (3.933 * Math.PI)) * 2;
            strafeDistanceFactor = 1.073;
        } else if (robotType == 1) {
            // Dog
            minTurnSpeed = 0.06;
            demoDriveSpeed = 1;
            demoStrafeSpeed = 1;
            demoTurnSpeed = 0.5;
            pulsesPerInch = (383.6 / (3.73 * Math.PI)) * 2;
            strafeDistanceFactor = 1.275;
        }
    }

    /**
     * Describe this function...
     */
    private void initMotors() {
        if (robotType == 2) {
            // Gobilda Surrogate
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (robotType == 1) {
            // Dog
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private void stowWhiskers() {
        servo1.setPosition(0);
        servo2.setPosition(0);
    }

    /**
     * Describe this function...
     */
    private void activateWhiskers() {
        servo1.setPosition(0.8);
        servo2.setPosition(0.8);
    }

    /**
     * Describe this function...
     */
    private double getHeading() {    //was float
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void initIMU() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    /**
     * Describe this function...
     */
    private double getError(double targetAngle) {
        double robotError;

        // determines the error between the target angle and the robot's current heading
        // targetAngle - Desired angle
        // returns Degrees in the range +/- 180
        // +ve error means the robot should turn LEFT (CCW) to reduce error.
        robotError = targetAngle - getHeading();
        // calculate error in -179 to +180 range
        while (robotError > 180) {
            robotError += -360;
        }
        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    /**
     * Describe this function...
     */
    private void goDrive(double speed, double distance, double targetHeading, int mode) {
        int startPosition;
        double adjSpeed;
        double workingError;
        double correction;

        if (opModeIsActive()) {
            if (mode == 0) {
                // correct heading if off too much
                // Remove the getError block to allow Java conversion
                if (getError(targetHeading) > 2) {
                    setHeading(targetHeading, demoTurnSpeed, 1, 5);
                }
            } else if (mode == 1) {
                // always set heading first
                setHeading(targetHeading, demoTurnSpeed, 1, 5);
            }
            startPosition = motor0.getCurrentPosition();
            motor0.setTargetPosition((int)(motor0.getCurrentPosition() + pulsesPerInch * distance));
            motor2.setTargetPosition((int)(motor2.getCurrentPosition() + pulsesPerInch * distance));
            motor1.setTargetPosition((int)(motor1.getCurrentPosition() + pulsesPerInch * distance));
            motor3.setTargetPosition((int)(motor3.getCurrentPosition() + pulsesPerInch * distance));
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setPower(0.1);
            motor2.setPower(0.1);
            motor1.setPower(0.1);
            motor3.setPower(0.1);
            while (motor0.isBusy() && motor2.isBusy() && motor1.isBusy() && motor3.isBusy() && opModeIsActive()) {
                adjSpeed = speed;
                workingError = getError(targetHeading);
                // adjust sign of error to match direction of travel
                workingError = workingError * (distance / Math.abs(distance));
                correction = 0.2 * (workingError / 10);
                correction = Math.min(Math.max(correction, speed * -0.2), speed * 0.2);
                if (Math.abs(adjSpeed) + Math.abs(correction) > 1) {
                    // recalculate if >1
                    adjSpeed = adjSpeed * (1 / (Math.abs(adjSpeed) + Math.abs(correction)));
                    correction = 0.2 * (workingError / 10);
                    correction = Math.min(Math.max(correction, adjSpeed * -0.2), adjSpeed * 0.2);
                }
                motor0.setPower(adjSpeed - correction);
                motor2.setPower(adjSpeed - correction);
                motor1.setPower(adjSpeed + correction);
                motor3.setPower(adjSpeed + correction);
                telemetry.addData("Error", JavaUtil.formatNumber(workingError, 2));
                telemetry.addData("Heading (Rot about Z)", getHeading());
                telemetry.addData("Correction", correction);
                telemetry.addData("Left Encoder", motor0.getCurrentPosition());
                telemetry.addData("Right Encoder", motor2.getCurrentPosition());
                telemetry.update();
                if (flagWhiskers != 0 && motor0.getCurrentPosition() > startPosition + pulsesPerInch * flagWhiskers) {
                    flagWhiskers = 0;
                    activateWhiskers();
                }
//                if (flagGrabber != 0 && motor0.getCurrentPosition() > startPosition + pulsesPerInch * flagGrabber) {
//                    flagGrabber = 0;
//                    closeGrabber();
//                }
            }
            motor0.setPower(0);
            motor2.setPower(0);
            motor1.setPower(0);
            motor3.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void goStrafe(double speed, double distance, double targetHeading, int mode) {
        //int startPosition;
        double adjSpeed;
        double workingError;
        double correction;

        distance = distance * strafeDistanceFactor;
        if (opModeIsActive()) {
            if (mode == 0) {
                // correct heading if off too much
                if (getError(targetHeading) > 2) {
                    setHeading(targetHeading, demoTurnSpeed, 1, 5);
                }
            } else if (mode == 1) {
                // always set heading first
                setHeading(targetHeading, demoTurnSpeed, 1, 5);
            }
            motor0.setTargetPosition((int)(motor0.getCurrentPosition() - pulsesPerInch * distance));
            motor1.setTargetPosition((int)(motor1.getCurrentPosition() + pulsesPerInch * distance));
            motor2.setTargetPosition((int)(motor2.getCurrentPosition() + pulsesPerInch * distance));
            motor3.setTargetPosition((int)(motor3.getCurrentPosition() - pulsesPerInch * distance));
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setPower(0.1);
            motor2.setPower(0.1);
            motor1.setPower(0.1);
            motor3.setPower(0.1);
            while (motor0.isBusy() && motor2.isBusy() && motor1.isBusy() && motor3.isBusy() && opModeIsActive()) {
                adjSpeed = speed;
                workingError = getError(targetHeading);
                // adjust sign of error to match direction of travel
                workingError = workingError * (distance / Math.abs(distance));
                correction = 0.2 * (workingError / 10);
                correction = Math.min(Math.max(correction, speed * -0.2), speed * 0.2);
                if (Math.abs(adjSpeed) + Math.abs(correction) > 1) {
                    // recalculate if >1
                    adjSpeed = adjSpeed * (1 / (Math.abs(adjSpeed) + Math.abs(correction)));
                    correction = 0.2 * (workingError / 10);
                    correction = Math.min(Math.max(correction, adjSpeed * -0.2), adjSpeed * 0.2);
                }
                motor0.setPower(adjSpeed - correction);
                motor2.setPower(adjSpeed - correction);
                motor1.setPower(adjSpeed + correction);
                motor3.setPower(adjSpeed + correction);
                telemetry.addData("Error", JavaUtil.formatNumber(workingError, 2));
                telemetry.addData("Heading (Rot about Z)", getHeading());
                telemetry.addData("Correction", correction);
                telemetry.addData("Left Encoder", motor0.getCurrentPosition());
                telemetry.addData("Right Encoder", motor2.getCurrentPosition());
                telemetry.update();
            }
            motor0.setPower(0);
            motor2.setPower(0);
            motor1.setPower(0);
            motor3.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void setHeading(double targetHeading, double maxTurnSpeed, double maxError, int confidence) {
        double workingError;
        int turnConfidence;
        double turnDirection;
        double turnSpeed;

        if (opModeIsActive()) {
            workingError = getError(targetHeading);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (Math.abs(workingError) < maxError) {
                turnConfidence = confidence;
            } else {
                turnConfidence = 0;
            }
            while (opModeIsActive() && turnConfidence < confidence) {
                turnDirection = -(workingError / Math.abs(workingError));
                turnSpeed = (Math.abs(workingError) / 45) * maxTurnSpeed;
                turnSpeed = Math.min(Math.max(turnSpeed, minTurnSpeed), maxTurnSpeed);
                telemetry.addData("Error", JavaUtil.formatNumber(workingError, 2));
                telemetry.addData("Current Heading", JavaUtil.formatNumber(getHeading(), 2));
                telemetry.addData("Target Heading", JavaUtil.formatNumber(targetHeading, 2));
                telemetry.addData("Turn Speed", JavaUtil.formatNumber(turnSpeed * turnDirection, 2));
                telemetry.update();
                if (Math.abs(workingError) < maxError) {
                    turnConfidence += 1;
                    motor0.setPower(0);
                    motor2.setPower(0);
                    motor1.setPower(0);
                    motor3.setPower(0);
                } else {
                    turnConfidence = 0;
                    motor0.setPower(turnSpeed * turnDirection);
                    motor2.setPower(turnSpeed * turnDirection);
                    motor1.setPower(-(turnSpeed * turnDirection));
                    motor3.setPower(-(turnSpeed * turnDirection));
                }
                workingError = getError(targetHeading);
            }
        }
    }
}