package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "A_Emmet01", group = "")
@Disabled
public class A_Emmet01a extends LinearOpMode {

    private DigitalChannel digital0;
    private DigitalChannel digital1;
    private DigitalChannel digital2;
    private DigitalChannel digital3;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftRear;
    private DcMotor motorRightRear;
    private DcMotor motorMast;
    private DcMotor motorJib;
    private Servo servoGrabber;
    private Servo servo1;
    private Servo servo2;


    private ElapsedTime Timer_Loop;
    //private double Time_Loop;

    private double Limit_DriveSpeed;

    private double Control_Y_Drive;
    private double Control_X_Strafe;
    private double Control_R_Rotate;

    private int State_Homing;

    private int Pos_Mast_Hold;
    private int Pos_Mast_Max_Global;
    private int Pos_Mast_Min_Global;
    private int Pos_Mast_Low_JibSafe;
    private int Pos_Mast_WiresSafe;
    private int Pos_Mast_Max;
    private int Pos_Mast_Min;
    private int Pos_Mast_Current;
    private double Power_Mast;

    private int Pos_Jib_Hold;
    private int Pos_Jib_Max_Global;
    private int Pos_Jib_Min_Global;
    private int Pos_Jib_Low_MastSafe;
    private int Pos_Jib_WiresSafe;
    private int Pos_Jib_Max;
    private int Pos_Jib_Min;
    private int Pos_Jib_Current;
    private double Power_Jib;

    private double Position_Servo0;

    private int Flag_Crane_IsHomed;
    private boolean Flag_Emmet = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        digital0 = hardwareMap.digitalChannel.get("digital0");
        digital1 = hardwareMap.digitalChannel.get("digital1");
        motorLeftFront = hardwareMap.dcMotor.get("motor0");
        motorRightFront = hardwareMap.dcMotor.get("motor1");
        motorLeftRear = hardwareMap.dcMotor.get("motor2");
        motorRightRear = hardwareMap.dcMotor.get("motor3");
        motorMast = hardwareMap.dcMotor.get("motor0B");
        motorJib = hardwareMap.dcMotor.get("motor1B");
        servoGrabber = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        digital2 = hardwareMap.digitalChannel.get("digital2");
        digital3 = hardwareMap.digitalChannel.get("digital3");

        Initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Telemetry_LoopStart();
                ReadControls();
                // Disallow some functions when homing
                if (State_Homing == 0) {
                    SetCraneLimits();
                    ControlMast();
                    ControlJib();
                    ControlServos();
                } else {
                    HomeCrane();
                }
                ControlDrivetrain();
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
        digital0.setMode(DigitalChannel.Mode.INPUT);
        digital1.setMode(DigitalChannel.Mode.INPUT);
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
        // Crane Constants
        Pos_Mast_Max_Global = 5825;
        Pos_Mast_Min_Global = 0;
        Pos_Jib_Max_Global = 3975;
        Pos_Jib_Min_Global = 0;
        Pos_Mast_Low_JibSafe = 1300;
        Pos_Jib_Low_MastSafe = 1500;
        Pos_Mast_WiresSafe = 2400;
        Pos_Jib_WiresSafe = 1150;
        // Flag Variables
        Flag_Crane_IsHomed = 0;
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
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private double CalculateLoopTime() {
        double Time_Loop = Timer_Loop.milliseconds();

        Timer_Loop.reset();
        return Time_Loop;
    }

    /**
     * Describe this function...
     */
    private void ReadControls() {
        if (gamepad1.left_bumper) {
            // Left Bumper = Slow
            Limit_DriveSpeed = 0.1;
        } else if (gamepad1.right_bumper) {
            // Right Bumper = Maximum
            Limit_DriveSpeed = 1;
        } else {
            // Otherwise Medium Speed
            Limit_DriveSpeed = 0.5;
        }
        Control_Y_Drive = -gamepad1.left_stick_y;
        Control_X_Strafe = gamepad1.left_stick_x;
        Control_R_Rotate = gamepad1.right_stick_x;
        telemetry.addData("DRIVE STRAFE ROTATE", JavaUtil.formatNumber(Control_Y_Drive, 2) + "   " + JavaUtil.formatNumber(Control_X_Strafe, 2) + "   " + JavaUtil.formatNumber(Control_R_Rotate, 2));
    }

    /**
     * Describe this function...
     */
    private void Telemetry_LoopStart() {
        if (Flag_Crane_IsHomed == 0) {
            telemetry.addData("Crane Status", "===>  NOT HOMED  <===");
        } else {
            telemetry.addData("Crane Status", "homed");
        }
        if (State_Homing == 0) {
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
        if (State_Homing == 0) {
            // Minimize loop time when homing by
            // not reading unnecessary sensors
            telemetry.addData("Limit Switches", digital0.getState() + "   " + digital1.getState() + "   " + digital2.getState() + "   " + digital3.getState());
            telemetry.addData("Encoders B", Pos_Mast_Current + "   " + Pos_Jib_Current);
        }
        telemetry.addData("Servo", JavaUtil.formatNumber(Position_Servo0, 2));
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
    private void ControlDrivetrain() {
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, maxPower;

        leftFrontPower = Control_Y_Drive + Control_X_Strafe + Control_R_Rotate;
        rightFrontPower = Control_Y_Drive - Control_X_Strafe - Control_R_Rotate;
        leftRearPower = Control_Y_Drive - Control_X_Strafe + Control_R_Rotate;
        rightRearPower = Control_Y_Drive + Control_X_Strafe - Control_R_Rotate;

        //find max and normalize
        maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));
        maxPower = Math.max(maxPower, 1);
        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightRearPower /= maxPower;

        //limit motors to max speed
        leftFrontPower *= Limit_DriveSpeed;
        rightFrontPower *= Limit_DriveSpeed;
        leftRearPower *= Limit_DriveSpeed;
        rightRearPower *= Limit_DriveSpeed;

        //set motor powers
        motorLeftFront.setPower(leftFrontPower);
        motorRightFront.setPower(rightFrontPower);
        motorLeftRear.setPower(leftRearPower);
        motorRightRear.setPower(rightRearPower);

    }
}
