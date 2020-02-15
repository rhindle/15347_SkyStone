package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "PID Turn Test", group = "")
public class PID_2wheel_Turn_Test extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;

    private BNO055IMU imu;

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.dcMotor.get("motor0");
        motorRight = hardwareMap.dcMotor.get("motor1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        initialize();

        if (opModeIsActive()) {
            boolean bumped = false;
            // Put run blocks here.
            while (opModeIsActive()) {

                addTelemetryLoopStart();
                if (gamepad1.left_bumper && !bumped) {
                    bumped = true;
                    rotate(90);
                } else if (gamepad1.right_bumper && !bumped) {
                    bumped = true;
                    rotate(-90);
                } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    bumped = false;
                }
                addTelemetryLoopEnd();
                telemetry.update();
            }
        }
    }

    private void rotate(double angle) {
        double newTarget = Math.round(getHeading()/Math.abs(angle))*Math.abs(angle)+angle;
        //autoTurnPID(newTarget,1,1,10);
        autoTurn(newTarget,1,0.5,10);
    }


    private void autoTurnPID(double targetHeading, double turnSpeedMaximum, double maxError, int confidence) {

        ElapsedTime timerTurnLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double timeTurnLoop;
        double turnSpeed;
        final double turnSpeedMinimum = 0.05;
        int confidenceCounter = 0;

        double currentError;
        double lastError;
        double iError = 0;
        double dError = 0;

//        double coeffKp = 0.06; //1.0/180.0; //0.0055555555555556‬;
//        double coeffKi = 0;// 0.00006; //0.00003;  /0.0003
//        double coeffKd = 2.5;//.5;//3;

        double coeffKp = 0.03; //1.0/180.0; //0.0055555555555556‬;
        double coeffKi = 0; // 0.00006; //0.00003;  /0.0003
        double coeffKd = 2; //2.5;//.5;//3;


        if (opModeIsActive()) {
            currentError = getError(targetHeading);
            if (Math.abs(currentError) < maxError) return;
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            timerTurnLoop.reset();

            while (opModeIsActive() && confidenceCounter < confidence) {

                timeTurnLoop = timerTurnLoop.milliseconds();
                timerTurnLoop.reset();

                lastError = currentError;
                currentError = getError(targetHeading);

                //iError += (currentError * timeTurnLoop);
                iError += (lastError + currentError) * 0.5 * timeTurnLoop;
                dError = (currentError - lastError) / timeTurnLoop;

                if (Math.abs(currentError) < maxError) {
                    confidenceCounter++;
                    turnSpeed = 0;
                } else {
                    confidenceCounter = 0;

                    turnSpeed = coeffKp * currentError;  // P
                    turnSpeed += coeffKi * iError;  // I
                    turnSpeed += coeffKd * dError;  // D

                    turnSpeed = Math.abs(turnSpeed);
                    turnSpeed = Math.min(turnSpeed, turnSpeedMaximum);  // Keep magnitude under turnSpeedMaximum
                    //turnSpeed = Math.max(turnSpeed, turnSpeedMinimum);  // Keep magnitude over turnSpeedMinimum
                    turnSpeed *= Math.signum(currentError);    // Restore the sign/direction

                    //turnSpeed = Math.abs(currentError) * coeffKp * turnSpeedMaximum + turnSpeedMinimum;  // Get scaled power magnitude
                    //turnSpeed = Math.min(turnSpeed, turnSpeedMaximum);    // Keep magnitude under turnSpeedMaximum
                    //turnSpeed *= Math.signum(currentError);    // Correct the sign/direction
                }

                //Output = kp * error + ki * errSum + kd * dErr;

                motorLeft.setPower(turnSpeed);
                motorRight.setPower(-turnSpeed);

                telemetry.addData("current heading", getHeading());
                telemetry.addData("target heading", targetHeading);
                telemetry.addData("error", currentError);
                telemetry.addData("turn speed", turnSpeed);
                telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(timeTurnLoop, 0));
                telemetry.update();
            }
        }
    }


    private void autoTurn(double targetHeading, double turnSpeedMaximum, double maxError, int confidence) {
        double currentError;
        double turnSpeed;
        int confidenceCounter = 0;
        double rampErrorLevel = 90; //180;
        final double turnSpeedMinimum = 0.025;

        if (opModeIsActive()) {
            currentError = getError(targetHeading);
            if (Math.abs(currentError) < maxError) return;
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && confidenceCounter < confidence) {

                currentError = getError(targetHeading);
                if (Math.abs(currentError) < maxError) {
                    confidenceCounter++;
                    turnSpeed = 0;
                } else {
                    confidenceCounter = 0;
                    turnSpeed = Math.abs(currentError) / rampErrorLevel * turnSpeedMaximum + turnSpeedMinimum;  // Get scaled power magnitude
                    turnSpeed = Math.min(turnSpeed, turnSpeedMaximum);    // Keep magnitude under turnSpeedMaximum
                    turnSpeed *= Math.signum(currentError);    // Correct the sign/direction
                }
                motorLeft.setPower(turnSpeed);
                motorRight.setPower(-turnSpeed);

                telemetry.addData("current heading", getHeading());
                telemetry.addData("target heading", targetHeading);
                telemetry.addData("error", currentError);
                telemetry.addData("turn speed", turnSpeed);
                telemetry.update();
            }
        }
    }


    private void addTelemetryLoopStart() {
        // Other debugging stuff
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        telemetry.addData("heading", getHeading());
    }

    private void addTelemetryLoopEnd() {
        // Add anything?
    }

    private double calculateLoopTime() {
        timeLoop = timerLoop.milliseconds();
        timerLoop.reset();
        return timeLoop;
    }

    private void initialize() {
        boolean imuOk = false;
        double heading;

        // Set motor directions and modes
        initMotors();
        initIMU();

        // Wait for Start...
        while (!isStarted()) {
            heading = getHeading();
            if (heading != 0) imuOk = true;
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to Start");
            telemetry.addData("Heading", heading);

            //telemetry.addData("IMU OK?", imuOk ? "TRUE" : "*** FALSE ***");
            if (!imuOk) {
                telemetry.addData("IMU OK?", "*** FALSE ***");
            } else if (Math.abs(heading)>0.2) {
                telemetry.addData("IMU OK?", "*** Moved Too Much - Init Again");
            } else {
                telemetry.addData("IMU OK?", "TRUE");
            }

            telemetry.update();
        }
    }

    private void initMotors() {
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

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

    private double getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

}