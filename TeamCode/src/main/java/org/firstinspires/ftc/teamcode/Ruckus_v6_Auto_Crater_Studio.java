package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Objects;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@Autonomous(name = "v6cleanv2speedautotomanousfacecratermsi (Blocks to Java)", group = "")
@Disabled
public class Ruckus_v6_Auto_Crater_Studio extends LinearOpMode {

    private DcMotor lifter;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private TfodRoverRuckus tfodRoverRuckus;
    private BNO055IMU imu;
    private VuforiaRoverRuckus vuforiaRoverRuckus;
    private Servo Test;
    private DcMotor sholderServo;

    double mineralPosition;
    ElapsedTime stallTimer;
    double lowSpeed;
    double highSpeed;
    double turnSpeed;
    double latchDirection;
    double loopCounter;
    ElapsedTime elapsedTime;
    String loopSpeedText;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        lifter = hardwareMap.dcMotor.get("lifter");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor ");
        tfodRoverRuckus = new TfodRoverRuckus();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        vuforiaRoverRuckus = new VuforiaRoverRuckus();
        Test = hardwareMap.servo.get("Test");
        sholderServo = hardwareMap.dcMotor.get("sholderServo");

        // Get the robot ready
        // (Start once robot is hung, level,
        // and camera aimed)
        initialize();
        // Wait for start
        // and find the gold mineral
        waitOnLatch();
        // Run phase...
        if (opModeIsActive()) {
            // Lower the robot off the lander
            getOffLander();
            // Sample the mineral
            sampleMineral();
            // Drive to the depot
            driveToDepot();
            // Drop team marker
            claimDepot();
            // Park in the crater
            driveToCrater();
            // Path is complete
            endGame();
        }

        tfodRoverRuckus.close();
        vuforiaRoverRuckus.close();
    }

    /**
     * Describe this function...
     */
    private void initialize() {
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopCounter = 0;
        loopSpeedText = "";
        // latchDirection: 0 = crater 1 = depot
        latchDirection = 0;
        // set some other constants
        lowSpeed = 0.6;
        highSpeed = 1;
        turnSpeed = 0.6;
        // Set motor directions and modes
        initMotors();
        // Because the lift mechanism began drifting,
        // need to find its zero position
        getLifterZero();
        // Set lifter motor to hold near zero
        holdLifterZero();
        // Init the TensorFlow vision library
        initTensorFlow();
        // Init the IMU for direction headings
        initIMU();
    }

    /**
     * Describe this function...
     */
    private void getLifterZero() {
        stallTimer = new ElapsedTime();
        stallTimer.reset();
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setTargetPosition(-10000);
        lifter.setPower(1);
        while (lifter.isBusy() && stallTimer.seconds() < 3) {
            telemetry.addData("Lifter Encoder Position", lifter.getCurrentPosition());
            telemetry.update();
        }
        lifter.setPower(0);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void goDrive(double speed, double distance) {
        double pulsesPerInch;

        if (opModeIsActive()) {
            pulsesPerInch = 1120 / (4 * Math.PI);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() + pulsesPerInch * distance));
            rightMotor.setTargetPosition((int) (rightMotor.getCurrentPosition() + pulsesPerInch * distance));
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            while ((leftMotor.isBusy() || rightMotor.isBusy()) && opModeIsActive()) {
                telemetry.addData("Heading (Rot about Z)", getHeading());
                telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
                telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    /**
     * Describe this function... things
     */
    private double findGold() {
        double leftRightOrCenter;
        //List recognitions;
        List <Recognition> recognitions;  //copied from Tom
        double goldMineralX;
        double silverMineral1X;
        double silverMineral2X;

        tfodRoverRuckus.activate();
        leftRightOrCenter = 0;
        // This will loop until Start is pressed
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Loops/Sec", calcLoopSpeed());
            float gotHeading = getHeading();
            if (gotHeading != 0) {
                telemetry.addData("Heading", gotHeading);
            } else {
                telemetry.addData("Heading", "ZERO (potential problem?)");
            }

      /* telemetry.addData("Heading (Rot about Z)", getHeading());
      if (getHeading()==0) {
        telemetry.addData("Alert", "IMU Problem?");
      } */
            telemetry.addData("Lifter Encoder Position", lifter.getCurrentPosition());
            // Put loop blocks here.
            recognitions = tfodRoverRuckus.getRecognitions();
            telemetry.addData("# Objects Recognized", recognitions.size());
            // At competition, this was set to 1.
            // Might work better as 0?
            if (recognitions.size() >= 0) {
                goldMineralX = -1;
                silverMineral1X = -1;
                silverMineral2X = -1;
                // TO_DO: Enter the type for variable named recognition
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals("Gold Mineral")) {
                        goldMineralX = recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = recognition.getLeft();
                    } else {
                        silverMineral2X = recognition.getLeft();
                    }
                }
                if (goldMineralX != -1) {
                    telemetry.addData("gold x", goldMineralX);
                    if (goldMineralX > 600) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        leftRightOrCenter = 1;
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        leftRightOrCenter = 2;
                    }
                } else {
                    telemetry.addData("Gold Mineral Position", "Left?");
                    leftRightOrCenter = 3;
                }
            }
            telemetry.update();
        }
        tfodRoverRuckus.deactivate();
        return leftRightOrCenter;
    }

    /**
     * Describe this function...
     */
    private void initMotors() {
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private String calcLoopSpeed() {
        loopCounter += 1;
        if (elapsedTime.milliseconds() > 1000) {
            loopSpeedText = JavaUtil.formatNumber(loopCounter / (elapsedTime.milliseconds() / 1000), 0);
            loopCounter = 0;
            elapsedTime.reset();
        }
        return loopSpeedText;
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
    private void holdLifterZero() {
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // 10 so it doesn't jam/stall at 0
        lifter.setTargetPosition(10);
        lifter.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void waitOnLatch() {
        mineralPosition = 0;
        // findGold will loop until Start pressed
        mineralPosition = findGold();
        // If no minerals seen, guess left
        if (mineralPosition == 0) {
            // Bug: At competition, this was set to 2 (center)!
            mineralPosition = 3;
        }
    }

    /**
     * Describe this function...
     */
    private void driveToDepot() {
        if (latchDirection == 0) {
            // --FACING CRATER--
            // Follow Safe Path!
            // Turn robot's back toward alliance wall
            goTurn(-95);
            // Drive reverse to wall
            // (used to be -51, but crashing into wall too hard)
            goDrive(highSpeed, -47);
            // Turn robot's back toward depot
            goTurn(-47);
            // Drive reverse to depot
            goDrive(highSpeed, -28);
        } else {
            // --FACING DEPOT--
            // Follow Safe Path!
            // Turn robot's back toward alliance wall
            // (front toward neutral wall at left)
            goTurn(85);
            // Drive forward past minerals
            goDrive(highSpeed, 18.5);
            // Turn perpendicular to neutral wall
            goTurn(45);
            // Drive forward to neutral wall
            goDrive(highSpeed, 24);
            // Turn robot's back toward depot
            goTurn(137);
            // Drive reverse to depot
            goDrive(highSpeed, -30);
        }
    }

    /**
     * Describe this function...
     */
    private void getOffLander() {
        if (opModeIsActive()) {
            stallTimer = new ElapsedTime();
            stallTimer.reset();
            lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lifter.setTargetPosition(4750);
            lifter.setPower(1);
            while (lifter.isBusy() && stallTimer.seconds() < 3 && opModeIsActive()) {
                telemetry.addData("Lifter Encoder Position", lifter.getCurrentPosition());
                telemetry.update();
            }
            lifter.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void sampleMineral() {
        // Go to "home" position
        goDrive(lowSpeed, 11);
        retractLifter();
        // sample right mineral
        if (mineralPosition == 1) {
            goTurn(-45);
            goDrive(lowSpeed, 20);
            // Return to "home" position
            goDrive(lowSpeed, -20);
        }
        // ...or sample middle mineral
        if (mineralPosition == 2) {
            goTurn(0);
            goDrive(lowSpeed, 15);
            // Return to "home" position
            goDrive(lowSpeed, -15);
        }
        // ...or sample left mineral
        if (mineralPosition == 3) {
            goTurn(45);
            goDrive(lowSpeed, 20);
            // Return to "home" position
            goDrive(lowSpeed, -20);
        }
        lifter.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void initTensorFlow() {
        vuforiaRoverRuckus.initialize("", VuforiaLocalizer.CameraDirection.BACK,
                true, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                0, 0, 0, 0, 0, 0, true);
        tfodRoverRuckus.initialize(vuforiaRoverRuckus, (float)0.4, true, true);
    }

    /**
     * Describe this function...
     */
    private void goTurn(double angle) {
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setHeading(angle, turnSpeed / 1, 25);
        setHeading(angle, turnSpeed / 4, 5);
        setHeading(angle, turnSpeed / 8, 1);
    }

    /**
     * Describe this function...
     */
    private void retractLifter() {
        if (opModeIsActive()) {
            lifter.setTargetPosition(10);
            lifter.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private float getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void claimDepot() {
        // This version drops the mineral by rotating a servo
        if (opModeIsActive()) {
            Test.setPosition(1);
            sleep(1600);
        }
    }

    /**
     * Describe this function...
     */
    private void setHeading(double targetHeading, double speed, double allowance) {
        float heading;
        double turnDirection;

        if (opModeIsActive()) {
            heading = getHeading();
            if (Math.abs(targetHeading - heading) > allowance) {
                // 1=left 0=right
                if (targetHeading < heading) {
                    turnDirection = 0;
                    leftMotor.setPower(speed);
                    rightMotor.setPower(-speed);
                }
                if (targetHeading > heading) {
                    turnDirection = 1;
                    leftMotor.setPower(-speed);
                    rightMotor.setPower(speed);
                }
                while (Math.abs(targetHeading - heading) > allowance && opModeIsActive()) {
                    heading = getHeading();
                    telemetry.addData("Current Heading", heading);
                    telemetry.addData("Target Heading", targetHeading);
                    telemetry.update();
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void claimDepotOld() {
        // This version drops mineral by lifting shoulder
        // This stalled in competition.
        // Use more power and/or add stall timer?
        if (opModeIsActive()) {
            sholderServo.setDirection(DcMotorSimple.Direction.REVERSE);
            sholderServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sholderServo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sholderServo.setTargetPosition(250);
            sholderServo.setPower(0.5);
            while (sholderServo.isBusy() && opModeIsActive()) {
                telemetry.addData("Shoulder Encoder", sholderServo.getCurrentPosition());
                telemetry.update();
            }
            sholderServo.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void driveToCrater() {
        if (latchDirection == 0) {
            // --FACING CRATER--
            // Realign robot's front toward alliance crater
            // (used to be -45, but robot drifting)
            goTurn(-40);
            // Drive forward to crater
            goDrive(highSpeed, 64);
        } else {
            // --FACING DEPOT--
            // Realign robot's front toward opposing crater
            goTurn(135);
            // Drive forward to crater
            goDrive(highSpeed, 58);
        }
    }

    /**
     * Describe this function...
     */
    private void endGame() {
        // Maybe useful for debugging
        while (opModeIsActive()) {
            // Display orientation info.
            telemetry.addData("Heading (Rot about Z)", getHeading());
            telemetry.addData("Left Encoder", leftMotor.getCurrentPosition());
            telemetry.addData("Right Encoder", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
