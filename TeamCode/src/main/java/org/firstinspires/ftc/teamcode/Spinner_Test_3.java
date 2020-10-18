package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "6W 2M Spinner Test 3", group = "")
//@Disabled
public class Spinner_Test_3 extends LinearOpMode {

    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    private DcMotorEx motorSpin;

    private BNO055IMU imu;

    private ElapsedTime timerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeLoop;

//    private ElapsedTime spinTimerLoop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime spinUpTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double spinUpTime;
//    private double spinTimeLoop;
//    double spinSpeed;
    double spinPower;
    double spinRPM;
    int spinEncoder;

    private double globalHeading;
    private int spinCount = 0;
    //private double loopTime;

    //private double storedHeading = 0;
    private double deltaHeading = 0;

    private boolean forza=true;
    private boolean spinModeVelocity=true;

    @Override
    public void runOpMode() {
//        motorLeft = hardwareMap.get(DcMotorEx.class, "motor0");
//        motorRight = hardwareMap.get(DcMotorEx.class, "motor1");
        motorSpin = hardwareMap.get(DcMotorEx.class, "motor3");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        initialize();

        if (opModeIsActive()) {
            boolean bumped = false;
            boolean ybumped = false;
            // Put run blocks here.
            while (opModeIsActive()) {

                globalHeading = getHeading();
                addTelemetryLoopStart();

                /*if (gamepad1.left_bumper && !bumped) {
                    bumped = true;
                    rotate(90);
                } else if (gamepad1.right_bumper && !bumped) {
                    bumped = true;
                    rotate(-90);
                } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    bumped = false;
                }*/

                //driveControls();

                spinControls();

/*                if (!gamepad1.right_bumper) bumped=false;

                //if (gamepad1.right_bumper) rotateToZero();

                if (gamepad1.dpad_left) {
                    //autoTurn(globalHeading+1.5,1,0.5,10);
                }

                if (gamepad1.dpad_right) {
                    //autoTurn(globalHeading-1.5,1,0.5,10);
                }

                if (gamepad1.left_bumper) {
                    deltaHeading = globalHeading;
                }

                if (!gamepad1.y) ybumped=false;
                if (!ybumped && gamepad1.y) {
                    ybumped=true;
                    forza=!forza;
                }
                telemetry.addData("forza", forza);

                //if (gamepad1.x) jogover(1);
                //if (gamepad1.a) jogover (-1);
*/

                addTelemetryLoopEnd();
                telemetry.update();
            }
        }
    }

    private void spinControls() {
        //145.6
        //int spinEncoder;

        /*spinTimeLoop = spinTimerLoop.milliseconds();
        if (spinTimeLoop > 1000){
            spinTimerLoop.reset();
            spinEncoder = motorSpin.getCurrentPosition();
            spinCount = spinEncoder - spinCount;
            spinSpeed = spinCount / spinTimeLoop * 1000 * 60 / 145.6 * 7;  //1:7 ratio
            spinCount = spinEncoder;
        }*/

        double spinMultiplier = 60 / 145.6 * 7;
        double spinSpeed2 = motorSpin.getVelocity() * spinMultiplier;

        if (spinSpeed2 > 5000 && spinUpTime==0) spinUpTime=spinUpTimer.milliseconds();

        if (gamepad2.left_stick_y != 0) spinPower = -gamepad2.left_stick_y;

        if (gamepad2.dpad_up) {
            spinPower += .01;
            if (spinPower > 1) spinPower = 1;
            spinRPM += 10;
        }
        if (gamepad2.dpad_down) {
            spinPower -= .01;
            if (spinPower < -1) spinPower = -1;
            spinRPM -= 10;
        }
        if (gamepad2.b) {
            spinPower = 0;
            spinRPM = 0;
        }
        if (gamepad2.y) {
            //spinModeVelocity = !spinModeVelocity;
        }
        if (gamepad2.x) {
            spinModeVelocity = true;
            spinRPM=5000;
            spinUpTimer.reset();
            spinUpTime = 0;
        }
        if (gamepad2.a) {
            spinModeVelocity = true;
            spinRPM=1000;
            //spinUpTimer.reset();
            //spinUpTime = 0;
        }


//        telemetry.addData("RPM", spinSpeed);
        telemetry.addData("Target RPM", spinRPM);
        telemetry.addData ("Reported RPM", spinSpeed2);
//        telemetry.addData("spinPower", spinPower);
//        telemetry.addData ("spinCount", spinCount);
        telemetry.addData ("encoder", spinEncoder);
        telemetry.addData ("spinUpTime", spinUpTime);

        if (!spinModeVelocity) {
            motorSpin.setPower(spinPower);
        } else {
            motorSpin.setVelocity(spinRPM / spinMultiplier);
        }

    }

/*    private void driveControls(){
        double driveSpeed, steer;
        double v0, v1, highvalue;

        if (forza) {
            if ((gamepad1.left_trigger==0 && gamepad1.right_trigger==0) || (gamepad1.left_trigger>0 & gamepad1.right_trigger>0)) {
                // neither pressed or both?  Zero out forward/reverse speed
                driveSpeed = 0;
            }
            else {
                //something is pressed; assign forward/reverse speed
                driveSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
            }
            steer = gamepad1.left_stick_x;
        }
        else {
            driveSpeed = -gamepad1.left_stick_y;
            steer = gamepad1.right_stick_x;
        }

        v0 = driveSpeed + steer;
        v1 = driveSpeed - steer;
        // now normalize them?
        highvalue=Math.max(Math.abs(v0), Math.abs(v1));
        highvalue=Math.max(1, highvalue);
        v0 /= highvalue;
        v1 /= highvalue;
        motorLeft.setPower(v0);
        motorRight.setPower(v1);
        telemetry.addData("v0", v0);
        telemetry.addData("v1", v1);
    }

    private void rotateToZero() {
        autoTurn(0+deltaHeading,1,0.5,10);
    }

    private void rotate(double angle) {
        double newTarget = Math.round(getHeading()/Math.abs(angle))*Math.abs(angle)+angle;

        //autoTurnPID(newTarget,1,1,10);
        autoTurn(newTarget,1,0.5,10);
    }

    private void jogover (int direction) {
        if (gamepad1.b) return;
        autoTurn (-45*Math.signum(direction)+deltaHeading, 1, 1, 1);
        if (gamepad1.b) return;
        autoDriveSimple(1,-12);
        if (gamepad1.b) return;
        autoTurn (0+deltaHeading, 1, 1, 1);
        if (gamepad1.b) return;
        autoDriveSimple(1,6);
        if (gamepad1.b) return;
        rotateToZero();
    }

    private void autoDriveSimple (double driveSpeed, double driveDistance) {
        double autoPulsesPerInch = (753.2 / (3.78 * Math.PI)) * 1;
        if (opModeIsActive()) {
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setTargetPosition((int) (motorLeft.getCurrentPosition() + driveDistance * autoPulsesPerInch));
            motorRight.setTargetPosition((int) (motorRight.getCurrentPosition() + driveDistance * autoPulsesPerInch));
            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft.setPower(driveSpeed);
            motorRight.setPower(driveSpeed);
            while (opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()) {
                if (gamepad1.b) break;
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
        }
    }

    private void autoTurn(double targetHeading, double turnSpeedMaximum, double maxError, int confidence) {
        // This is ramped (partially proportional)
        double currentError;
        double turnSpeed;
        int confidenceCounter = 0;
        double rampErrorLevel = 90; //180;
        final double turnSpeedMinimum = 0.025;

        if (opModeIsActive()) {
            currentError = getError(targetHeading, globalHeading);
            if (Math.abs(currentError) < maxError) return;
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive() && confidenceCounter < confidence) {

                globalHeading = getHeading();
                currentError = getError(targetHeading, globalHeading);
                if (Math.abs(currentError) < maxError) {
                    confidenceCounter++;
                    turnSpeed = 0;
                } else {
                    confidenceCounter = 0;
                    turnSpeed = Math.abs(currentError) / rampErrorLevel * turnSpeedMaximum + turnSpeedMinimum;  // Get scaled power magnitude
                    turnSpeed = Math.min(turnSpeed, turnSpeedMaximum);    // Keep magnitude under turnSpeedMaximum
                    turnSpeed *= Math.signum(currentError);    // Correct the sign/direction
                }
                motorLeft.setPower(-turnSpeed);
                motorRight.setPower(turnSpeed);

                telemetry.addData("current heading", globalHeading);
                telemetry.addData("target heading", targetHeading);
                telemetry.addData("error", currentError);
                telemetry.addData("turn speed", turnSpeed);
                telemetry.update();

                if (gamepad1.b) break;
            }
        }
    }

*/

    private void addTelemetryLoopStart() {
        // Other debugging stuff
        //loopTime=calculateLoopTime();
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calculateLoopTime(), 0));
        //telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(loopTime, 0));
        telemetry.addData("heading", globalHeading);
        telemetry.addData("deltaheading", deltaHeading);
        telemetry.addData("effectiveheading", globalHeading-deltaHeading);
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
 /*       motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/

        motorSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    // This costs 7 ms per use.  Use once per loop only if possible.
    private double getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getError(double targetAngle) {
        return getError(targetAngle, getHeading());
    }
    public double getError(double targetAngle, double heading) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - heading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }



}