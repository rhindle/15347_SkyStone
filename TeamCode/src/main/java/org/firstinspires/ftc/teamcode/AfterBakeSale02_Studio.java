package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "AfterBakeSale02 (Android Studio)", group = "")
public class AfterBakeSale02_Studio extends LinearOpMode {

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

    double maxDriveSpeed;
    double LBpressed;
    double ServoPosition;
    double ServoSpeed;
    ElapsedTime loopTimer;
    //List loopTimeList;
    //List<Integer> loopTimeList;
    double loopTimeArray[] = new double[10];

    /**
     * Describe this function...
     */
    private void bulkTelemetry() {
        telemetry.addData("Heading", getHeading());
        telemetry.addData("Loop time (lps)", JavaUtil.formatNumber(calcLoopTime(), 0));
        telemetry.addData("Limit Switches", digital0.getState() + "   " + digital1.getState());
        telemetry.addData("Encoders A", motor0.getCurrentPosition() + "   " + motor1.getCurrentPosition() + "   " + motor2.getCurrentPosition() + "   " + motor3.getCurrentPosition());
        telemetry.addData("Encoders B", motor0B.getCurrentPosition() + "   " + motor1B.getCurrentPosition() + "   " + motor2B.getCurrentPosition() + "   " + motor3B.getCurrentPosition());
    }

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
        maxDriveSpeed = 1;
        maxDriveSpeed = 1;
        LBpressed = 0;
        ServoPosition = 0.5;
        ServoSpeed = 0.05;
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Display orientation info.
                bulkTelemetry();
                controlModifiers();
                controlMast();
                ServoControl();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private double calcLoopTime() {
        double temp;
        double temp2;

        //loopTimeList = JavaUtil.createListWith(JavaUtil.inListGetSublist(loopTimeList, JavaUtil.AtMode.FROM_START, 0, JavaUtil.AtMode.FROM_START, 3), loopTimer.milliseconds());

        //LEIGH NEW
        for (int i = 0; i < loopTimeArray.length-1; i++) {
            loopTimeArray[i] = loopTimeArray[i+1];
        }
        loopTimeArray[loopTimeArray.length-1]=loopTimer.milliseconds();
        temp2=loopTimer.milliseconds();

        loopTimer.reset();
        temp = 0;
        // TODO: Enter the type for variable named i
        //for (int i : loopTimeList) {
        //  temp += i;
        //}
        for (int i = 0; i < loopTimeArray.length; i++) {
            temp += loopTimeArray[i];
        }
        temp /= loopTimeArray.length;
        //temp = 1000 / (temp / loopTimeArray.length);
        //temp2 = 1000 / temp2;
        return temp;//2;
    }

    /**
     * Describe this function...
     */
    private void initialize() {
        ElapsedTime elapsedTime;

        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        //loopTimeList = JavaUtil.createListWith(1000, 1000, 1000, 1000, 1000);
        // Set motor directions and modes
        initMotors();
        digital0.setMode(DigitalChannel.Mode.INPUT);
        digital1.setMode(DigitalChannel.Mode.INPUT);
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
    private float getHeading() {
        Orientation angles;

        // Get absolute orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Describe this function...
     */
    private void ServoControl() {
        // Use gamepad A and B to open close servo
        if (gamepad1.a) {
            ServoPosition += ServoSpeed;
        }
        if (gamepad1.b) {
            ServoPosition += -ServoSpeed;
        }
        // Keep Servo position in valid range
        ServoPosition = Math.min(Math.max(ServoPosition, 0), 1);
        servo0.setPosition(ServoPosition);
        telemetry.addData("Servo", ServoPosition);
    }

    /**
     * Describe this function...
     */
    private void controlModifiers() {
        if (gamepad1.y) {
            maxDriveSpeed = 0.5;
        } else {
            maxDriveSpeed = 1;
        }
    }

    /**
     * Describe this function...
     */
    private void controlMast() {
        double Z_Lift;

        Z_Lift = gamepad1.right_trigger - gamepad1.left_trigger;
        telemetry.addData("Mast Power (raw)", Double.parseDouble(JavaUtil.formatNumber(Z_Lift, 2)));
        if (digital0.getState()) {
            Z_Lift = Math.min(Math.max(Z_Lift, -1), 0);
        }
        if (gamepad1.left_bumper) {
            LBpressed += 1;
        } else {
            if (LBpressed >= 5) {
                motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            LBpressed = 0;
            if (motor0B.getCurrentPosition() <= 0) {
                Z_Lift = Math.min(Math.max(Z_Lift, 0), 1);
            }
            if (motor0B.getCurrentPosition() <= 2000 && Z_Lift < 0) {
                Z_Lift = Z_Lift * ((double)motor0B.getCurrentPosition() / 2000);
            }
        }
        telemetry.addData("Mast Power (constrained)", Double.parseDouble(JavaUtil.formatNumber(Z_Lift, 2)));
        telemetry.addData("Encoder0B (Mast)", motor0B.getCurrentPosition());
        motor0B.setPower(Z_Lift);
    }

    /**
     * Describe this function...
     */
    private void DriveControl() {
        double Y_Drive;
        double X_Strafe;
        double R_Rotate;
        double v1;
        double v2;
        double v3;
        double v0;
        double highvalue;

        Y_Drive = -Math.pow(gamepad1.left_stick_y, 1);
        X_Strafe = Math.pow(gamepad1.left_stick_x, 1);
        R_Rotate = Math.pow(gamepad1.right_stick_x, 1);
        v0 = Y_Drive + X_Strafe + R_Rotate;
        v1 = (Y_Drive - X_Strafe) - R_Rotate;
        v2 = (Y_Drive - X_Strafe) + R_Rotate;
        v3 = (Y_Drive + X_Strafe) - R_Rotate;
        highvalue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v1), Math.abs(v2), Math.abs(v3), 1));
        motor0.setPower((v0 / highvalue) * maxDriveSpeed);
        motor1.setPower((v1 / highvalue) * maxDriveSpeed);
        motor2.setPower((v2 / highvalue) * maxDriveSpeed);
        motor3.setPower((v3 / highvalue) * maxDriveSpeed);
        telemetry.addData("DRIVE", Double.parseDouble(JavaUtil.formatNumber(Y_Drive, 2)));
        telemetry.addData("STRAFE", Double.parseDouble(JavaUtil.formatNumber(X_Strafe, 2)));
        telemetry.addData("ROTATE", Double.parseDouble(JavaUtil.formatNumber(R_Rotate, 2)));
        telemetry.addData("v0", Double.parseDouble(JavaUtil.formatNumber(v0, 2)));
        telemetry.addData("v1", Double.parseDouble(JavaUtil.formatNumber(v1, 2)));
        telemetry.addData("v2", Double.parseDouble(JavaUtil.formatNumber(v2, 2)));
        telemetry.addData("v3", Double.parseDouble(JavaUtil.formatNumber(v3, 2)));
        telemetry.addData("Encoder0", motor0.getCurrentPosition());
        telemetry.addData("Encoder1", motor1.getCurrentPosition());
        telemetry.addData("Encoder2", motor2.getCurrentPosition());
        telemetry.addData("Encoder3", motor3.getCurrentPosition());
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
}
