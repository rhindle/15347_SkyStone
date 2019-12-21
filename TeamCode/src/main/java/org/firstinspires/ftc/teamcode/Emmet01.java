package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Emmet01", group = "")
public class Emmet01 extends LinearOpMode {

    private DigitalChannel digitalMastHigh;
    private DigitalChannel digitalJibHigh;
    private DigitalChannel digitalMastLow;
    private DigitalChannel digitalJibLow;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftRear;
    private DcMotor motorRightRear;
    private DcMotor motorMast;
    private DcMotor motorJib;
    private Servo servoGrabber;
    private Servo servoLeftWhisker;
    private Servo servoRightWhisker;


    private ElapsedTime Timer_Loop;
    private double Time_Loop;

    private double driveSpeedLimit;

    private double controlDrive;
    private double controlStrafe;
    private double controlRotate;

    private int homingState;

    private final int mastPositionMax = 6200;
    private final int mastPositionMin = 0;
    private final int mastPositionJibSafe = 200;
    private final int mastPositionBridgeSafe = 300;

    private final int jibPositionMax = 3900;
    private final int jibPositionMin = 0;
    private final int jibPositionPark = 350;
    private final int jibPositionGrab = 1000;
    private final int jibPositionPlace = 1675;

    private int mastPositionHold;
    //private int Pos_Mast_WiresSafe;
    //private int Pos_Mast_Max;
    //private int Pos_Mast_Min;
    private int mastPositionCurrent;
    private double controlMastPower;

    private int jibPositionHold;
    //private int Pos_Jib_Low_MastSafe;
    //private int Pos_Jib_WiresSafe;
    //private int Pos_Jib_Max;
    private int jibPositionTempMin;
    private int jibPositionCurrent;
    private double controlJibPower;

    private double grabberPosition;

    private boolean flagCraneIsHomed = false;
    private boolean flagEmmetIsCoolerThanYou = true;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        digitalMastHigh = hardwareMap.digitalChannel.get("digital0");
        digitalJibHigh = hardwareMap.digitalChannel.get("digital1");
        digitalMastLow = hardwareMap.digitalChannel.get("digital2");
        digitalJibLow = hardwareMap.digitalChannel.get("digital3");
        motorLeftFront = hardwareMap.dcMotor.get("motor0");
        motorRightFront = hardwareMap.dcMotor.get("motor1");
        motorLeftRear = hardwareMap.dcMotor.get("motor2");
        motorRightRear = hardwareMap.dcMotor.get("motor3");
        motorMast = hardwareMap.dcMotor.get("motor0B");
        motorJib = hardwareMap.dcMotor.get("motor1B");
        servoGrabber = hardwareMap.servo.get("servo0");
        servoLeftWhisker = hardwareMap.servo.get("servo1");
        servoRightWhisker = hardwareMap.servo.get("servo2");


        Initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Telemetry_LoopStart();
                ReadControls();
                // Disallow some functions when homing
                if (homingState == 0) {
                    SetCraneLimits();
                    ControlMast();
                    ControlJib();
                    ControlServos();
                } else {
                    HomeCrane();
                }
                controlDrivetrain();
                Telemetry_LoopEnd();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Initialize() {
        // Initialize variables
        SetVariables();
        // Set motor directions and modes
        InitMotors();
        // Set digital i/o
        digitalMastHigh.setMode(DigitalChannel.Mode.INPUT);
        digitalJibHigh.setMode(DigitalChannel.Mode.INPUT);
        // Wait for Start...
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void SetVariables() {
        Timer_Loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Describe this function...
     */
    private void InitMotors() {
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorMast.setDirection(DcMotorSimple.Direction.FORWARD);
        motorMast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorMast.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorJib.setDirection(DcMotorSimple.Direction.REVERSE);
        motorJib.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorJib.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorJib.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoGrabber.setDirection(Servo.Direction.FORWARD);
        servoLeftWhisker.setDirection(Servo.Direction.FORWARD);
        servoRightWhisker.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private double CalculateLoopTime() {
        Time_Loop = Timer_Loop.milliseconds();
        Timer_Loop.reset();
        return Time_Loop;
    }

    /**
     * Describe this function...
     */
    private void ReadControls() {
        if (gamepad1.left_bumper) {
            // Left Bumper = Slow
            driveSpeedLimit = 0.1;
        } else if (gamepad1.right_bumper) {
            // Right Bumper = Maximum
            driveSpeedLimit = 1;
        } else {
            // Otherwise Medium Speed
            driveSpeedLimit = 0.5;
        }
        controlDrive = -gamepad1.left_stick_y;
        controlStrafe = gamepad1.left_stick_x;
        controlRotate = gamepad1.right_stick_x;
        telemetry.addData("DRIVE STRAFE ROTATE", JavaUtil.formatNumber(controlDrive, 2) + "   " + JavaUtil.formatNumber(controlStrafe, 2) + "   " + JavaUtil.formatNumber(controlRotate, 2));
    }

    /**
     * Describe this function...
     */
    private void Telemetry_LoopStart() {
        if (flagCraneIsHomed == false) {
            telemetry.addData("Crane Status", "===>  NOT HOMED  <===");
        } else {
            telemetry.addData("Crane Status", "homed");
        }
        if (homingState == 0) {
            // Minimize loop time when homing by
            // not reading unnecessary sensors
        }
        // Other debugging stuff
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(CalculateLoopTime(), 0));
    }

    /**
     * Describe this function...
     */
    private void Telemetry_LoopEnd() {
        // !! Disable Sensor Polling for more speed !!
        // Best to read only values needed each loop
        // and store them in variables
        if (homingState == 0) {
            // Minimize loop time when homing by
            // not reading unnecessary sensors
            telemetry.addData("Limit Switches", digitalMastHigh.getState() + "   " + digitalJibHigh.getState() + "   " + digitalMastLow.getState() + "   " + digitalJibLow.getState());
            telemetry.addData("Encoders B", mastPositionCurrent + "   " + jibPositionCurrent);
        }
        telemetry.addData("Servo", JavaUtil.formatNumber(grabberPosition, 2));
    }

    /**
     * Describe this function...
     */
    private void SetCraneLimits() {
    }

    /**
     * Describe this function...
     */
    private void HomeCrane() {
    }

    /**
     * Describe this function...
     */
    private void ControlServos() {
    }

    /**
     * Describe this function...
     */
    private void ControlMast() {
    }

    /**
     * Describe this function...
     */
    private void ControlJib() {
    }

    /**
     * Describe this function...
     */
    private void controlDrivetrain() {
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, driveMaxPower;

        //setting power levels based on drive controls
        leftFrontPower = controlDrive + controlStrafe + controlRotate;
        rightFrontPower = controlDrive - controlStrafe - controlRotate;
        leftRearPower = controlDrive - controlStrafe + controlRotate;
        rightRearPower = controlDrive + controlStrafe - controlRotate;

        //find max and normalize
        driveMaxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));
        driveMaxPower = Math.max(driveMaxPower, 1);
        leftFrontPower /= driveMaxPower;
        rightFrontPower /= driveMaxPower;
        leftRearPower /= driveMaxPower;
        rightRearPower /= driveMaxPower;

        //limit motors to max speed
        leftFrontPower *= driveSpeedLimit;
        rightFrontPower *= driveSpeedLimit;
        leftRearPower *= driveSpeedLimit;
        rightRearPower *= driveSpeedLimit;

        //set motor powers
        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftRear.setPower(leftRearPower);
        motorRightRear.setPower(rightRearPower);

    }
}
