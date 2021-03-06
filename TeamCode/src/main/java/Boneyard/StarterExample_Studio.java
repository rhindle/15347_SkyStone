package Boneyard;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "StarterExample (Studio)", group = "")
@Disabled
public class StarterExample_Studio extends LinearOpMode {

    private DigitalChannel digital0;
    private DigitalChannel digital1;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor0B;
    private DcMotor motor1B;
    private Servo servo0;
    private Servo servo1;
    private Servo servo2;
    private DigitalChannel digital2;
    private DigitalChannel digital3;

    private ElapsedTime Timer_Loop;
    private double Time_Loop;

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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        digital0 = hardwareMap.digitalChannel.get("digital0");
        digital1 = hardwareMap.digitalChannel.get("digital1");
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor0B = hardwareMap.dcMotor.get("motor0B");
        motor1B = hardwareMap.dcMotor.get("motor1B");
        servo0 = hardwareMap.servo.get("servo0");
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
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0B.setDirection(DcMotorSimple.Direction.FORWARD);
        motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1B.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo0.setDirection(Servo.Direction.FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
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
    }
}
