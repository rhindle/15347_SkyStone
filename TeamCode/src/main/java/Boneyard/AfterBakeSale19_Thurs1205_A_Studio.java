package Boneyard;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "After Bake Sale 19 (A) - Thurs 1205 (Studio)", group = "")
@Disabled
public class AfterBakeSale19_Thurs1205_A_Studio extends LinearOpMode {

    private Servo servo0;
    private Servo servo1;
    private Servo servo2;
    private DigitalChannel digital0;
    private DigitalChannel digital1;
    private BNO055IMU imu;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor0B;
    private DcMotor motor1B;
    private DigitalChannel digital3;
    private DigitalChannel digital2;
    private DcMotor motor2B;
    private DcMotor motor3B;

    double Limit_DriveSpeed;
    double Limit_MastSpeed;
    double Limit_JibSpeed;
    int Limit_HomingTime;

    int Pos_Mast_Hold;
    int Pos_Mast_Max_Global;
    int Pos_Mast_Min_Global;
    int Pos_Mast_Low_JibSafe;
    int Pos_Mast_WiresSafe;
    int Pos_Mast_Max;
    int Pos_Mast_Min;
    int Pos_Mast_Current;
    double Power_Mast;

    int Pos_Jib_Hold;
    int Pos_Jib_Max_Global;
    int Pos_Jib_Min_Global;
    int Pos_Jib_Low_MastSafe;
    int Pos_Jib_WiresSafe;
    int Pos_Jib_Max;
    int Pos_Jib_Min;
    int Pos_Jib_Current;
    double Power_Jib;

    double Power_M0;
    double Power_M1;
    double Power_M2;
    double Power_M3;
    double Power_Drive_Max;

    double Control_Y_Drive;
    double Control_X_Strafe;
    double Control_R_Rotate;

    int Flag_LeftBumper_IsPressed;
    int Flag_RightBumper_IsPressed;
    int Flag_Grabber_IsParked;
    int Flag_Mast_IsHolding;
    int Flag_Jib_IsHolding;
    int Flag_CancelAction;
    int Flag_Crane_IsHomed;
    int Flag_MoverToggle;
    int State_Homing;

    double Position_Servo0;
    double Speed_Servo0;
    double Pos_FoundationMover_Up;
    double Pos_FoundationMover_Down;
    double Pos_Grabber_Open;
    double Pos_Grabber_Closed;

    ElapsedTime Timer_Elapsed;
    ElapsedTime Timer_Loop;
    ElapsedTime Timer_Homing;
    double Time_Loop;
    double Time_Homing;

    String message;

    /**
     * Describe this function...
     */
    private void BulkTelemetry1() {
        if (Flag_Crane_IsHomed == 0) {
            telemetry.addData("Crane Status", "===>  NOT HOMED  <===");
        } else {
            telemetry.addData("Crane Status", "homed");
        }
        if (State_Homing == 0) {
            // Minimize loop time when homing by
            // not reading unnecessary sensors
            // Display orientation info.
            telemetry.addData("Heading", JavaUtil.formatNumber(GetHeading(), 2));
        }
        // Other debugging stuff
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(CalcLoopTime(), 0));
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        digital0 = hardwareMap.digitalChannel.get("digital0");
        digital1 = hardwareMap.digitalChannel.get("digital1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor0B = hardwareMap.dcMotor.get("motor0B");
        motor1B = hardwareMap.dcMotor.get("motor1B");
        digital3 = hardwareMap.digitalChannel.get("digital3");
        digital2 = hardwareMap.digitalChannel.get("digital2");
        motor2B = hardwareMap.dcMotor.get("motor2B");
        motor3B = hardwareMap.dcMotor.get("motor3B");

        Initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            // disable homing function until limit switches added
            servo0.setDirection(Servo.Direction.FORWARD);
            servo1.setDirection(Servo.Direction.FORWARD);
            servo2.setDirection(Servo.Direction.REVERSE);
            while (opModeIsActive()) {
                BulkTelemetry1();
                InputModifiers();
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
                BulkTelemetry2();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void HomingDebugDelay() {
        sleep(50);
    }

    /**
     * Describe this function...
     */
    private void Initialize() {
        // Init variables
        SetVariablesAndConstants();
        // Set motor directions and modes
        InitMotors();
        // Set digital i/o
        digital0.setMode(DigitalChannel.Mode.INPUT);
        digital1.setMode(DigitalChannel.Mode.INPUT);
        // IMU
        InitIMU();
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Heading (Rot about Z)", GetHeading());
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void SetVariablesAndConstants() {
        Limit_DriveSpeed = 1;
        Limit_MastSpeed = 1;
        Limit_JibSpeed = 0.5;
        Limit_HomingTime = 15;
        Position_Servo0 = 0.5;
        Speed_Servo0 = 0.05;
        Timer_Elapsed = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer_Loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer_Homing = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Pos_Mast_Hold = 0;
        Pos_Jib_Hold = 0;
        Flag_Grabber_IsParked = 1;
        Flag_LeftBumper_IsPressed = 0;
        Flag_Mast_IsHolding = 0;
        Flag_Jib_IsHolding = 0;
        Flag_CancelAction = 0;
        Flag_Crane_IsHomed = 0;
        Flag_MoverToggle = 0;
        State_Homing = 0;
        SetCraneConstants();
        Pos_FoundationMover_Up = 0;
        Pos_FoundationMover_Down = 1;
        Pos_Grabber_Open = 0.6;
        Pos_Grabber_Closed = 0.4;
    }

    /**
     * Describe this function...
     */
    private void InitIMU() {
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
    }

    /**
     * Describe this function...
     */
    private void HomeCrane() {
        // Only run if homing requested
        if (State_Homing != 0) {
            if (State_Homing == 1) {
                // Reset the homing timeout timer
                Timer_Homing.reset();
            }
            // Do this every loop
            if (1 == 1) {
                // Read the motor encoders only ONCE PER LOOP
                // (except for the holds, which need freshest info)
                Pos_Mast_Current = motor0B.getCurrentPosition();
                Pos_Jib_Current = motor1B.getCurrentPosition();
                // How much time has elapsed in homing?
                Time_Homing = Timer_Homing.seconds();
                message = "Homing";
                // Need to cancel?
                if (gamepad2.b || Time_Homing > Limit_HomingTime) {
                    State_Homing = 9999;
                }
            }
            // Initialize
            if (State_Homing == 1) {
                // Put the grabber servo in a safe position
                servo0.setPosition(0.6);
                // Make sure the motors are stopped
                motor0B.setPower(0);
                motor1B.setPower(0);
                // Clear the "homed" flag
                Flag_Crane_IsHomed = 0;
                Flag_Mast_IsHolding = 0;
                Flag_Jib_IsHolding = 0;
                // Advance the state counter
                State_Homing += 1;
                // (set up for next state)
                // Move mast a minimum safe distance up
                motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor0B.setPower(1);
            }
            // Move mast a minimum safe distance up
            if (State_Homing == 2) {
                message = "Raising Mast";
                if (digital0.getState() || Pos_Mast_Current >= Pos_Mast_Low_JibSafe) {
                    motor0B.setPower(0);
                    // ...and hold it there
                    motor0B.setTargetPosition(motor0B.getCurrentPosition());
                    motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor0B.setPower(0.05);
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // Home the jib - Rough
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor1B.setPower(-1);
                }
            }
            // Home the jib - Rough
            if (State_Homing == 3) {
                message = "Homing Jib - Rough";
                if (digital3.getState() || Pos_Jib_Current < -9999) {
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Pos_Jib_Current = 0;
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // ...and extend for take two
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor1B.setPower(1);
                }
            }
            // ...and extend for take two
            if (State_Homing == 4) {
                message = "Homing Jib ";
                if (Pos_Jib_Current > 200) {
                    motor1B.setPower(0);
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // Home the jib - Precise
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor1B.setPower(-0.25);
                }
            }
            // Home the jib - Precise
            if (State_Homing == 5) {
                message = "Homing Jib - Precise";
                if (digital3.getState() || Pos_Jib_Current < -9999) {
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Pos_Jib_Current = 0;
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // Extend jib to a minimum safe distance
                    motor1B.setPower(0);
                    motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor1B.setPower(1);
                }
            }
            // Extend jib to a minimum safe distance
            if (State_Homing == 6) {
                message = "Extending Jib";
                if (Pos_Jib_Current >= Pos_Jib_Low_MastSafe) {
                    motor1B.setPower(0);
                    // ...and hold it there
                    motor1B.setTargetPosition(motor1B.getCurrentPosition());
                    motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor1B.setPower(0.05);
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // Home the Mast - Rough
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor0B.setPower(-0.5);
                }
            }
            // Home the Mast - Rough
            if (State_Homing == 7) {
                message = "Homing Mast - Rough";
                if (digital2.getState() || Pos_Mast_Current < -9999) {
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Pos_Mast_Current = 0;
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // ...and extend for take two
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor0B.setPower(0.25);
                }
            }
            // ...and extend for take two
            if (State_Homing == 8) {
                message = "Homing Mast";
                if (Pos_Mast_Current > 250) {
                    motor0B.setPower(0);
                    State_Homing += 1;
                    // (set up for next state)
                    // Home the mast - Precise
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor0B.setPower(-0.2);
                }
            }
            // Home the mast - Precise
            if (State_Homing == 9) {
                message = "Homing Mast - Precise";
                if (digital2.getState() || Pos_Mast_Current < -9999) {
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Pos_Mast_Current = 0;
                    HomingDebugDelay();
                    State_Homing += 1;
                    // (set up for next state)
                    // Extend mast up to jib safe position
                    motor0B.setPower(0);
                    motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor0B.setPower(1);
                }
            }
            // Extend mast up to jib safe position
            if (State_Homing == 10) {
                message = "Extending Mast";
                if (Pos_Mast_Current > Pos_Mast_Low_JibSafe) {
                    motor0B.setPower(0);
                    // ...and hold it there
                    motor0B.setTargetPosition(motor0B.getCurrentPosition());
                    motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor0B.setPower(0.05);
                    HomingDebugDelay();
                    State_Homing += 1;
                }
            }
            // All done!
            if (State_Homing == 11) {
                Flag_Crane_IsHomed = 1;
                State_Homing = 0;
            }
            // Was homing cancelled?
            if (State_Homing == 9999) {
                // Hold both motors wherever they are
                motor0B.setPower(0);
                motor0B.setTargetPosition(motor0B.getCurrentPosition());
                motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor0B.setPower(0.05);
                motor1B.setPower(0);
                motor1B.setTargetPosition(motor1B.getCurrentPosition());
                motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1B.setPower(0.05);
                HomingDebugDelay();
                State_Homing = 0;
            }
            // Do this every loop
            if (1 == 1) {
                telemetry.addData("Status", "~~~~~  HOMING IN PROGRESS ~~~~~");
                telemetry.addData("Message", message);
                telemetry.addData("Homing State #", State_Homing);
                telemetry.addData("Mast Encoder", Pos_Mast_Current);
                telemetry.addData("Jib Encoder", Pos_Jib_Current);
                telemetry.addData("Homing Time (seconds)", JavaUtil.formatNumber(Time_Homing, 0));
            }
        }
    }

    /**
     * Describe this function...
     */
    private void BulkTelemetry2() {
        // !! Disable Sensor Polling for more speed !!
        // Best to read only values needed each loop
        // and store them in variables
        if (State_Homing == 0) {
            // Minimize loop time when homing by
            // not reading unnecessary sensors
            telemetry.addData("Limit Switches", digital0.getState() + "   " + digital1.getState() + "   " + digital2.getState() + "   " + digital3.getState());
            telemetry.addData("Encoders A", motor0.getCurrentPosition() + "   " + motor1.getCurrentPosition() + "   " + motor2.getCurrentPosition() + "   " + motor3.getCurrentPosition());
            telemetry.addData("Encoders B", Pos_Mast_Current + "   " + Pos_Jib_Current + "   " + motor2B.getCurrentPosition() + "   " + motor3B.getCurrentPosition());
        }
        telemetry.addData("Servo", JavaUtil.formatNumber(Position_Servo0, 2));
    }

    /**
     * Describe this function...
     */
    private void SetCraneConstants() {
        Pos_Mast_Max_Global = 5975;
        Pos_Mast_Min_Global = 0;
        Pos_Jib_Max_Global = 3950;
        Pos_Jib_Min_Global = 0;
        Pos_Mast_Low_JibSafe = 1300;
        Pos_Jib_Low_MastSafe = 1500;
        Pos_Mast_WiresSafe = 2400;
        Pos_Jib_WiresSafe = 1150;
    }

    /**
     * Describe this function...
     */
    private void InputModifiers() {
        if (gamepad1.left_bumper) {
            Limit_DriveSpeed = 0.5;
        } else {
            Limit_DriveSpeed = 1;
        }
        if (State_Homing == 0) {
            // Only allow homing if not already homing
            if (gamepad2.left_bumper && gamepad2.right_bumper) {
                State_Homing = 1;
            }
        } else {
            // Limit drive speed when homing
            Limit_DriveSpeed = 0.25;
        }
    }

    /**
     * Describe this function...
     */
    private float GetHeading() {
        Orientation HeadingAngles;

        // Get absolute orientation
        // Costly in time; try to read only once per loop
        HeadingAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return HeadingAngles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private double CalcLoopTime() {
        Time_Loop = Timer_Loop.milliseconds();
        Timer_Loop.reset();
        return Time_Loop;
    }

    /**
     * Describe this function...
     */
    private void ControlJib() {
        Power_Jib = gamepad2.right_stick_x;
        Power_Jib = Math.min(Math.max(Power_Jib, -Limit_JibSpeed), Limit_JibSpeed);
        telemetry.addData("Jib Power (raw)", Double.parseDouble(JavaUtil.formatNumber(Power_Jib, 2)));
        // apply position limitations
        if (!gamepad2.right_bumper) {
            if (Pos_Jib_Current > Pos_Jib_Max && Power_Jib > 0 || Pos_Jib_Current < Pos_Jib_Min && Power_Jib < 0) {
                Power_Jib = 0;
            }
            Flag_RightBumper_IsPressed = 0;
        } else {
            Power_Jib = Math.min(Math.max(Power_Jib, -Limit_JibSpeed * 0.2), Limit_JibSpeed * 0.2);
            Flag_RightBumper_IsPressed += 1;
        }
        // JibFlag 0=was driving, 1=holding
        if (Power_Jib == 0) {
            // If power = 0, hold instead
            if (Flag_Jib_IsHolding == 0) {
                motor1B.setPower(0);
                Flag_RightBumper_IsPressed = 0;
                sleep(25);
                Flag_Jib_IsHolding = 1;
                Pos_Jib_Hold = Pos_Jib_Current;
                motor1B.setTargetPosition(Pos_Jib_Hold);
                motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1B.setPower(0.05);
            } else {
                telemetry.addData("Jib Holding at", Pos_Jib_Hold);
            }
        } else {
            // If power != 0, drive with constraints
            if (Flag_Jib_IsHolding == 1) {
                Flag_RightBumper_IsPressed = 0;
                Flag_Jib_IsHolding = 0;
                motor1B.setPower(0);
                motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // react to upper limit switch
            if (digital1.getState()) {
                Power_Jib = Math.min(Math.max(Power_Jib, -Limit_JibSpeed), 0);
            }
            // react to lower limit switch
            if (digital3.getState()) {
                Power_Jib = Math.min(Math.max(Power_Jib, 0), Limit_JibSpeed);
            }
            // slow down near lower limit
            if (!gamepad2.right_bumper) {
                if (Pos_Jib_Current - Pos_Jib_Min <= 2000 * Limit_JibSpeed && Power_Jib < 0) {
                    Power_Jib = Power_Jib * ((Pos_Jib_Current - Pos_Jib_Min) / (2000 * Limit_JibSpeed));
                }
            }
            telemetry.addData("Jib Power (constrained)", Double.parseDouble(JavaUtil.formatNumber(Power_Jib, 2)));
            motor1B.setPower(Power_Jib);
        }
    }

    /**
     * Describe this function...
     */
    private void ControlDrivetrain() {
        Control_Y_Drive = -Math.pow(gamepad1.left_stick_y, 1);
        Control_X_Strafe = Math.pow(gamepad1.left_stick_x, 1);
        Control_R_Rotate = Math.pow(gamepad1.right_stick_x, 1);
        telemetry.addData("DRIVE STRAFE ROTATE", JavaUtil.formatNumber(Control_Y_Drive, 2) + "   " + JavaUtil.formatNumber(Control_X_Strafe, 2) + "   " + JavaUtil.formatNumber(Control_R_Rotate, 2));
        Power_M0 = Control_Y_Drive + Control_X_Strafe + Control_R_Rotate;
        Power_M1 = (Control_Y_Drive - Control_X_Strafe) - Control_R_Rotate;
        Power_M2 = (Control_Y_Drive - Control_X_Strafe) + Control_R_Rotate;
        Power_M3 = (Control_Y_Drive + Control_X_Strafe) - Control_R_Rotate;
        telemetry.addData("M0 M1 M2 M3 (raw)", JavaUtil.formatNumber(Power_M0, 2) + "   " + JavaUtil.formatNumber(Power_M1, 2) + "   " + JavaUtil.formatNumber(Power_M2, 2) + "   " + JavaUtil.formatNumber(Power_M3, 2));
        Power_Drive_Max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(Power_M0), Math.abs(Power_M1), Math.abs(Power_M2), Math.abs(Power_M3), 1));
        Power_M0 = (Power_M0 / Power_Drive_Max) * Limit_DriveSpeed;
        Power_M1 = (Power_M1 / Power_Drive_Max) * Limit_DriveSpeed;
        Power_M2 = (Power_M2 / Power_Drive_Max) * Limit_DriveSpeed;
        Power_M3 = (Power_M3 / Power_Drive_Max) * Limit_DriveSpeed;
        telemetry.addData("M0 M1 M2 M3 (mod)", JavaUtil.formatNumber(Power_M0, 2) + "   " + JavaUtil.formatNumber(Power_M1, 2) + "   " + JavaUtil.formatNumber(Power_M2, 2) + "   " + JavaUtil.formatNumber(Power_M3, 2));
        if (1 == 1) {
            motor0.setPower(Power_M0);
            motor1.setPower(Power_M1);
            motor2.setPower(Power_M2);
            motor3.setPower(Power_M3);
        }
    }

    /**
     * Describe this function...
     */
    private void ControlMast() {
        Power_Mast = -gamepad2.left_stick_y;
        Power_Mast = Math.min(Math.max(Power_Mast, -Limit_MastSpeed), Limit_MastSpeed);
        telemetry.addData("Mast Power (raw)", Double.parseDouble(JavaUtil.formatNumber(Power_Mast, 2)));
        // apply position limitations
        if (!gamepad2.left_bumper) {
            if (Pos_Mast_Current > Pos_Mast_Max && Power_Mast > 0 || Pos_Mast_Current < Pos_Mast_Min && Power_Mast < 0) {
                Power_Mast = 0;
            }
            Flag_LeftBumper_IsPressed = 0;
        } else {
            Power_Mast = Math.min(Math.max(Power_Mast, -Limit_MastSpeed * 0.2), Limit_MastSpeed * 0.2);
            Flag_LeftBumper_IsPressed += 1;
        }
        // MastFlag 0=was driving, 1=holding
        if (Power_Mast == 0) {
            // If power = 0, hold instead
            if (Flag_Mast_IsHolding == 0) {
                motor0B.setPower(0);
                Flag_LeftBumper_IsPressed += 1;
                sleep(25);
                Flag_Mast_IsHolding = 1;
                Pos_Mast_Hold = Pos_Mast_Current;
                motor0B.setTargetPosition(Pos_Mast_Hold);
                motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor0B.setPower(0.05);
            } else {
                telemetry.addData("Mast Holding at", Pos_Mast_Hold);
            }
        } else {
            // If power != 0, drive with constraints
            if (Flag_Mast_IsHolding == 1) {
                Flag_LeftBumper_IsPressed = 0;
                Flag_Mast_IsHolding = 0;
                motor0B.setPower(0);
                motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // react to upper limit switch
            if (digital0.getState()) {
                Power_Mast = Math.min(Math.max(Power_Mast, -Limit_MastSpeed), 0);
            }
            // react to lower limit switch
            if (digital2.getState()) {
                Power_Mast = Math.min(Math.max(Power_Mast, 0), Limit_MastSpeed);
            }
            // slow down near lower limit
            if (!gamepad2.left_bumper) {
                if (Pos_Mast_Current - Pos_Mast_Min <= 2000 * Limit_MastSpeed && Power_Mast < 0) {
                    Power_Mast = Power_Mast * ((Pos_Mast_Current - Pos_Mast_Min) / (2000 * Limit_MastSpeed));
                }
            }
            telemetry.addData("Mast Power (constrained)", Double.parseDouble(JavaUtil.formatNumber(Power_Mast, 2)));
            motor0B.setPower(Power_Mast);
        }
    }

    /**
     * Describe this function...
     */
    private void ControlServos() {
        if (State_Homing == 0) {
            // Only allow control of grabber while
            // NOT homing
            // Use gamepad A and B to open close servo
            if (gamepad1.a) {
                Position_Servo0 += Speed_Servo0;
            }
            if (gamepad1.b) {
                Position_Servo0 += -Speed_Servo0;
            }
            if (gamepad2.a) {
                Position_Servo0 = Pos_Grabber_Closed;
            }
            if (gamepad2.b) {
                Position_Servo0 = Pos_Grabber_Open;
            }
            // Keep Servo position in valid range
            Position_Servo0 = Math.min(Math.max(Position_Servo0, 0), 1);
            servo0.setPosition(Position_Servo0);
        }
        if (State_Homing == 0) {
            // Only allow control of foundation mover
            // when NOT homing
            // Use gamepad 2 UP to activate
            // Use flag variable to minimize time
            // by not always sending servo position
            if (gamepad2.dpad_up) {
                if (Flag_MoverToggle == 0) {
                    Flag_MoverToggle = 1;
                    servo1.setPosition(Pos_FoundationMover_Down);
                    servo2.setPosition(Pos_FoundationMover_Down);
                }
            } else {
                if (Flag_MoverToggle == 1) {
                    Flag_MoverToggle = 0;
                    servo1.setPosition(Pos_FoundationMover_Up);
                    servo2.setPosition(Pos_FoundationMover_Up);
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void SetCraneLimits() {
        // Start by using the global maxima
        Pos_Mast_Max = Pos_Mast_Max_Global;
        Pos_Mast_Min = Pos_Mast_Min_Global;
        Pos_Jib_Max = Pos_Jib_Max_Global;
        Pos_Jib_Min = Pos_Jib_Min_Global;
        // Read the encoder positions
        // Use this variable elsewhere to avoid re-reading!
        Pos_Mast_Current = motor0B.getCurrentPosition();
        Pos_Jib_Current = motor1B.getCurrentPosition();
        // If mast has raised enough, flag grabber deployed
        if (Pos_Mast_Current > Pos_Mast_Low_JibSafe) {
            Flag_Grabber_IsParked = 0;
        }
        // Mast is low, grabber not deployed: jib can't move
        if (Pos_Mast_Current < Pos_Mast_Low_JibSafe && Flag_Grabber_IsParked == 1) {
            // (impossible to satisfy)
            Pos_Jib_Max = -9999;
            Pos_Jib_Min = 9999;
        }
        // Mast is low, jib is extended: jib can't retract
        if (Pos_Mast_Current < Pos_Mast_Low_JibSafe && Pos_Jib_Current > Pos_Jib_Low_MastSafe - 200) {
            Pos_Jib_Min = Pos_Jib_Low_MastSafe;
        }
        // Mast is high: jib can't retract (avoid wire pinch)
        if (Pos_Mast_Current > Pos_Mast_WiresSafe) {
            Pos_Jib_Min = Pos_Jib_WiresSafe;
        }
        // Jib is retracted too far, limit mast movement up
        if (Pos_Jib_Current < Pos_Jib_WiresSafe) {
            Pos_Mast_Max = Pos_Mast_WiresSafe;
        }
        // Jib is retracted too far, limit mast movement down
        if (Pos_Jib_Current < Pos_Jib_Low_MastSafe && Flag_Grabber_IsParked == 0) {
            Pos_Mast_Min = Pos_Mast_Low_JibSafe;
        }
    }

    /**
     * Describe this function...
     */
    private void Old_Boneyard_BumperOverrides() {
        if (Flag_RightBumper_IsPressed >= 5) {
            motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1B.setTargetPosition(0);
            motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1B.setPower(0.05);
        }
        if (Flag_RightBumper_IsPressed >= 5) {
            motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (Flag_LeftBumper_IsPressed >= 5) {
            motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0B.setTargetPosition(0);
            motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0B.setPower(0.05);
        }
        if (Flag_LeftBumper_IsPressed >= 5) {
            motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
