package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "AfterBakeSale05 (Android Studio)", group = "")
public class AfterBakeSale05_Studio extends LinearOpMode {

    private DigitalChannel digital0;
    private DigitalChannel digital1;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor0B;
    private DcMotor motor1B;
    private DcMotor motor2B;
    private DcMotor motor3B;
    private BNO055IMU imu;
    private Servo servo0;

  /*double maxDriveSpeed;
  double LBpressed;
  double ServoPosition;
  double ServoSpeed;
  ElapsedTime loopTimer;
  //List loopTimeList;
  //List<Integer> loopTimeList;
  double loopTimeArray[] = new double[10];*/


    double Limit_DriveSpeed ;
    double Position_Servo0;
    double Speed_Servo0;
    int Held_LeftBumper;
    int Held_RightBumper;
    int Flag_Jib;
    int Flag_Mast;
    int Hold_Jib;
    int Hold_Mast;
    ElapsedTime Timer_Elapsed;
    ElapsedTime Timer_Loop;
    double Power_M0;
    double Power_M1;
    double Power_M2;
    double Power_M3;
    double Power_Jib;
    double Power_Mast;
    double HighMotorPower;
    double X_Strafe;
    double Y_Drive;
    double R_Rotate;


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
        motor2B = hardwareMap.dcMotor.get("motor2B");
        motor3B = hardwareMap.dcMotor.get("motor3B");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        servo0 = hardwareMap.servo.get("servo0");

        initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                BulkTelemetry();
                controlModifiers();
                MastControl();
                JibControl();
                ServoControl();
                DriveControl();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void setVariablesAndConstants() {
        Limit_DriveSpeed = 1;
        Held_LeftBumper = 0;
        Position_Servo0 = 0.5;
        Speed_Servo0 = 0.05;
        Hold_Mast = 0;
        Hold_Jib = 0;
        Flag_Mast = 0;
        Flag_Jib = 0;
        Timer_Elapsed = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer_Loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    /**
     * Describe this function...
     */
    private void BulkTelemetry() {
        // Display orientation info.
        telemetry.addData("Heading", getHeading());
        // Other debugging stuff
        telemetry.addData("Loop time (ms)", JavaUtil.formatNumber(calcLoopTime(), 0));
        // !! Disable Sensor Polling for more speed !!
        telemetry.addData("Limit Switches", digital0.getState() + "   " + digital1.getState());
        telemetry.addData("Encoders A", motor0.getCurrentPosition() + "   " + motor1.getCurrentPosition() + "   " + motor2.getCurrentPosition() + "   " + motor3.getCurrentPosition());
        telemetry.addData("Encoders B", motor0B.getCurrentPosition() + "   " + motor1B.getCurrentPosition() + "   " + motor2B.getCurrentPosition() + "   " + motor3B.getCurrentPosition());
        telemetry.addData("Servo", JavaUtil.formatNumber(Position_Servo0, 2));
    }

    /**
     * Describe this function...
     */
    private double calcLoopTime() {
        double Time_Loop;
        Time_Loop = Timer_Loop.milliseconds();
        Timer_Loop.reset();
        return Time_Loop;
    }

    /**
     * Describe this function...
     */
  /*private void calcLoopTime_Old() {
    List_LoopTime = JavaUtil.createListWith(JavaUtil.inListGetSublist(List_LoopTime, JavaUtil.AtMode.FROM_START, 0, JavaUtil.AtMode.FROM_START, 3), Timer_Loop.milliseconds());
    Timer_Loop.reset();
    Time_Loop = 0;
    // TODO: Enter the type for variable named i
    for (UNKNOWN_TYPE i : List_LoopTime) {
      Time_Loop += i;
    }
    Time_Loop = 1000 / (Time_Loop / 5);
  }*/

    /**
     * Describe this function...
     */
    private float getHeading() {
        Orientation HeadingAngles;

        // Get absolute orientation
        HeadingAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return HeadingAngles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void initialize() {
        // Init variables
        setVariablesAndConstants();
        // Set motor directions and modes
        initMotors();
        // Set digital i/o
        digital0.setMode(DigitalChannel.Mode.INPUT);
        digital1.setMode(DigitalChannel.Mode.INPUT);
        // IMU
        initIMU();
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("Heading (Rot about Z)", getHeading());
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void controlModifiers() {
        if (gamepad1.y) {
            Limit_DriveSpeed = 0.5;
        } else {
            Limit_DriveSpeed = 1;
        }
    }

    /**
     * Describe this function...
     */
    private void JibControl() {
        Power_Jib = gamepad2.right_stick_x;
        telemetry.addData("Jib Power (raw)", Double.parseDouble(JavaUtil.formatNumber(Power_Jib, 2)));
        // JibFlag 0=was driving, 1=holding
        if (Power_Jib == 0) {
            if (Flag_Jib == 0) {
                motor1B.setPower(0);
                Held_RightBumper = 0;
                //sleep(25);
                Flag_Jib = 1;
                Hold_Jib = motor1B.getCurrentPosition();
                motor1B.setTargetPosition(Hold_Jib);
                motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor1B.setPower(0.05);
            } else {
                telemetry.addData("Jib Holding at", Hold_Jib);
                if (gamepad2.right_bumper) {
                    Held_RightBumper += 1;
                } else {
                    if (Held_RightBumper >= 5) {
                        motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor1B.setTargetPosition(0);
                        motor1B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor1B.setPower(0.05);
                    }
                    Held_RightBumper = 0;
                }
            }
        } else {
            if (Flag_Jib == 1) {
                Held_RightBumper = 0;
                Flag_Jib = 0;
                motor1B.setPower(0);
                motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
            }
            if (digital1.getState()) {
                Power_Jib = Math.min(Math.max(Power_Jib, -1), 0);
            }
            if (gamepad2.right_bumper) {
                Held_RightBumper += 1;
            } else {
                if (Held_RightBumper >= 5) {
                    motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                Held_RightBumper = 0;
                if (motor1B.getCurrentPosition() <= 0) {
                    Power_Jib = Math.min(Math.max(Power_Jib, 0), 1);
                }
                if (motor1B.getCurrentPosition() <= 2000 && Power_Jib < 0) {
                    Power_Jib = Power_Jib * ((double)motor1B.getCurrentPosition() / 2000);
                }
            }
            telemetry.addData("Jib Power (constrained)", Double.parseDouble(JavaUtil.formatNumber(Power_Jib, 2)));
            motor1B.setPower(Power_Jib);
        }
    }

    /**
     * Describe this function...
     */
    private void DriveControl() {
        Y_Drive = -Math.pow(gamepad1.left_stick_y, 1);
        X_Strafe = Math.pow(gamepad1.left_stick_x, 1);
        R_Rotate = Math.pow(gamepad1.right_stick_x, 1);
        telemetry.addData("DRIVE STRAFE ROTATE", JavaUtil.formatNumber(Y_Drive, 2) + "   " + JavaUtil.formatNumber(X_Strafe, 2) + "   " + JavaUtil.formatNumber(R_Rotate, 2));
        Power_M0 = Y_Drive + X_Strafe + R_Rotate;
        Power_M1 = (Y_Drive - X_Strafe) - R_Rotate;
        Power_M2 = (Y_Drive - X_Strafe) + R_Rotate;
        Power_M3 = (Y_Drive + X_Strafe) - R_Rotate;
        telemetry.addData("M0 M1 M2 M3 (raw)", JavaUtil.formatNumber(Power_M0, 2) + "   " + JavaUtil.formatNumber(Power_M1, 2) + "   " + JavaUtil.formatNumber(Power_M2, 2) + "   " + JavaUtil.formatNumber(Power_M3, 2));
        HighMotorPower = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(Power_M0), Math.abs(Power_M1), Math.abs(Power_M2), Math.abs(Power_M3), 1));
        Power_M0 = (Power_M0 / HighMotorPower) * Limit_DriveSpeed;
        Power_M1 = (Power_M1 / HighMotorPower) * Limit_DriveSpeed;
        Power_M2 = (Power_M2 / HighMotorPower) * Limit_DriveSpeed;
        Power_M3 = (Power_M3 / HighMotorPower) * Limit_DriveSpeed;
        telemetry.addData("M0 M1 M2 M3 (mod)", JavaUtil.formatNumber(Power_M0, 2) + "   " + JavaUtil.formatNumber(Power_M1, 2) + "   " + JavaUtil.formatNumber(Power_M2, 2) + "   " + JavaUtil.formatNumber(Power_M3, 2));
    }

    /**
     * Describe this function...
     */
    private void MastControl() {
        Power_Mast = -gamepad2.left_stick_y;
        telemetry.addData("Mast Power (raw)", Double.parseDouble(JavaUtil.formatNumber(Power_Mast, 2)));
        // MastFlag 0=was driving, 1=holding
        if (Power_Mast == 0) {
            if (Flag_Mast == 0) {
                motor0B.setPower(0);
                Held_LeftBumper += 1;
                //sleep(25);
                Flag_Mast = 1;
                Hold_Mast = motor0B.getCurrentPosition();
                motor0B.setTargetPosition(Hold_Mast);
                motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor0B.setPower(0.05);
            } else {
                telemetry.addData("Mast Holding at", Hold_Mast);
                if (gamepad2.left_bumper) {
                    Held_LeftBumper += 1;
                } else {
                    if (Held_LeftBumper >= 5) {
                        motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor0B.setTargetPosition(0);
                        motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor0B.setPower(0.05);
                    }
                    Held_LeftBumper = 0;
                }
            }
        } else {
            if (Flag_Mast == 1) {
                Held_LeftBumper = 0;
                Flag_Mast = 0;
                motor0B.setPower(0);
                motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
            }
            if (digital0.getState()) {
                Power_Mast = Math.min(Math.max(Power_Mast, -1), 0);
            }
            if (gamepad2.left_bumper) {
                Held_LeftBumper += 1;
            } else {
                if (Held_LeftBumper >= 5) {
                    motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                Held_LeftBumper = 0;
                if (motor0B.getCurrentPosition() <= 0) {
                    Power_Mast = Math.min(Math.max(Power_Mast, 0), 1);
                }
                if (motor0B.getCurrentPosition() <= 2000 && Power_Mast < 0) {
                    Power_Mast = Power_Mast * ((double)motor0B.getCurrentPosition() / 2000);
                }
            }
            telemetry.addData("Mast Power (constrained)", Double.parseDouble(JavaUtil.formatNumber(Power_Mast, 2)));
            motor0B.setPower(Power_Mast);
        }
    }

    /**
     * Describe this function...
     */
    private void ServoControl() {
        // Use gamepad A and B to open close servo
        if (gamepad1.a) {
            Position_Servo0 += Speed_Servo0;
        }
        if (gamepad1.b) {
            Position_Servo0 += -Speed_Servo0;
        }
        // Keep Servo position in valid range
        Position_Servo0 = Math.min(Math.max(Position_Servo0, 0), 1);
        servo0.setPosition(Position_Servo0);
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
    private void initMotors() {
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
        motor1B.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1B.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
