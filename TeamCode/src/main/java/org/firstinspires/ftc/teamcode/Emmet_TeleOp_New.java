package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Emmet's TeleOp", group = "")
public class Emmet_TeleOp_New extends LinearOpMode {

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
    private Servo servoCapstone;


    private ElapsedTime Timer_Loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double Time_Loop;
    private ElapsedTime timerHoming = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double timeHoming;
    private final int homingTimeLimit = 10;

    private double driveSpeedLimit;
    private final double driveSpeedDefault = 0.35;

    private double controlDrive;
    private double controlStrafe;
    private double controlRotate;

    private int homingState;

    private final int mastPositionMax = 6200;
    private final int mastPositionMin = 0;
    private final int mastPositionJibSafe = 200;
    private final int mastPositionBridgeSafe = 300;

    private final int jibPositionMax = 3829;
    private final int jibPositionMin = 0;
    private final int jibPositionPark = 350;
    private final int jibPositionGrab = 1000;
    private final int jibPositionPlace = 1675;

    private final int mastPresetHeights[] = {0, mastPositionBridgeSafe, 1100, 2400, 3700, 5000, 6200};

    private int mastPositionHold;
    private int mastPositionCurrent;
    private double controlMastPower;

    private int jibPositionHold;
    private int jibPositionCurrent;
    private double controlJibPower;

    private double grabberPosition;
    private final double grabberOpen = 0.6;
    private final double grabberClosed = 0.4;
    private final double grabberSafe = 0.8;
    private final double grabberHoming = 0.6;

    //whiskers
    private double whiskerPosition;
    private final double whiskerUp = 0;
    private final double whiskerDown = 0.8;
    private final double whiskerSpeedLimit = 0.275; //0.35, was 0.2

    //capstone
    private final double capstoneUp = 0.1;
    private final double capstoneDown = 0.65;
    private final double capstoneMove = 0.03;
    private double capstonePosition = capstoneUp;

    private boolean flagCraneIsHomed = false;
    private boolean flagMastHolding = false;
    private boolean flagJibHolding = false;
    private boolean flagEmmetIsCoolerThanYou = true;
    private  boolean flagPresetRequested = false;

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
        servoCapstone = hardwareMap.servo.get("servo5");

        initialize();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                Telemetry_LoopStart();
                readControls();
                // Don't allow some functions when homing
                if (homingState == 0) {
                    controlMast();
                    controlJib();
                    controlServos();
                } else {
                    homeCrane();
                }
                controlDrivetrain();
                Telemetry_LoopEnd();
                telemetry.update();
            }
        }
    }

    private void initialize() {
        // Set motor directions and modes
        initMotors();
        grabberPosition = grabberSafe;
        // Set digital i/o
        digitalMastHigh.setMode(DigitalChannel.Mode.INPUT);
        digitalJibHigh.setMode(DigitalChannel.Mode.INPUT);
        digitalMastLow.setMode(DigitalChannel.Mode.INPUT);
        digitalJibLow.setMode(DigitalChannel.Mode.INPUT);
        // Wait for Start...
        while (!isStarted()) {
            // Prompt user to press start buton.
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }
    }

    private void initMotors() {
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
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        servoCapstone.setDirection(Servo.Direction.REVERSE);
    }

    private double CalculateLoopTime() {
        Time_Loop = Timer_Loop.milliseconds();
        Timer_Loop.reset();
        return Time_Loop;
    }

    private void readControls() {
//        if (gamepad1.left_bumper) {
//            // Left Bumper = Slow
//            driveSpeedLimit = 0.1;
//        } else if (gamepad1.right_bumper) {
//            // Right Bumper = Maximum
//            driveSpeedLimit = 1;
//        } else {
//            // Otherwise Medium Speed
//            driveSpeedLimit = 0.5;
//        }

        // speed limiting
        if (gamepad1.left_trigger > 0) {
            driveSpeedLimit = driveSpeedDefault - gamepad1.left_trigger * (driveSpeedDefault - 0.15);
        } else if (gamepad1.right_trigger > 0) {
            driveSpeedLimit = driveSpeedDefault + gamepad1.right_trigger * (1 - driveSpeedDefault);
        } else {
            driveSpeedLimit = driveSpeedDefault;
        }
        if (homingState != 0) {
            driveSpeedLimit = Math.min(driveSpeedLimit, 0.2);
        }
//        if (mastPositionCurrent > 1500 || jibPositionCurrent > 2500) {
//            driveSpeedLimit = Math.min(driveSpeedLimit, 0.2);
//        }

        // new- don't use crane speed limit when whiskers down
        if (mastPositionCurrent > 1500 || jibPositionCurrent > 2500) {
            if (whiskerPosition != whiskerDown) {
                driveSpeedLimit = Math.min(driveSpeedLimit, 0.2);
            }
        }

        telemetry.addData("Speed Limit", driveSpeedLimit);

        //rest of controls
        controlDrive = -gamepad1.left_stick_y;
        controlStrafe = gamepad1.left_stick_x;
        controlRotate = gamepad1.right_stick_x;
        telemetry.addData("DRIVE STRAFE ROTATE", JavaUtil.formatNumber(controlDrive, 2) + "   " + JavaUtil.formatNumber(controlStrafe, 2) + "   " + JavaUtil.formatNumber(controlRotate, 2));

        controlMastPower = -gamepad2.left_stick_y;
        controlJibPower = gamepad2.right_stick_x;

        //only allow these controls if it's not homing
        if (homingState == 0) {
            if (gamepad2.left_bumper && gamepad2.right_bumper) homingState = 1;
            if (gamepad2.b) grabberPosition = grabberOpen;
            if (gamepad2.a) grabberPosition = grabberClosed;
            if (gamepad2.back) {
                whiskerPosition = whiskerDown;
            } else {
                whiskerPosition = whiskerUp;
            }
        }
        if (homingState == 0 && flagCraneIsHomed) {
            if (gamepad2.x) readyGrabber();
            if (gamepad2.y) multiStageGrab();
            if (gamepad2.dpad_left) parkGrabber();
            if (gamepad2.dpad_right) moveGrabberToFoundation();
            if (gamepad2.dpad_up && !flagPresetRequested) {
                flagPresetRequested = true;
                moveUpPresetHeights();
            }
            if (gamepad2.dpad_down && !flagPresetRequested) {
                flagPresetRequested = true;
                moveDownPresetHeights();
            }
        }
        if (!gamepad2.dpad_up && !gamepad2.dpad_down) flagPresetRequested = false;

        if(gamepad1.a) {
            capstonePosition = capstonePosition + capstoneMove;
            capstonePosition = Math.min(capstonePosition, capstoneDown);
        }

        if (gamepad1.b) {
            capstonePosition = capstonePosition - capstoneMove;
            capstonePosition = Math.max(capstonePosition, capstoneUp);
        }

    }

    private void multiStageGrab() {
        if (grabberPosition != grabberClosed) {
            if (Math.abs(mastPositionCurrent) < 50){
                if (jibPositionCurrent > jibPositionGrab - 50 && jibPositionCurrent - jibPositionGrab < 500) {
                    //make sure robot is stopped
                    motorLeftFront.setPower(0);
                    motorRightFront.setPower(0);
                    motorLeftRear.setPower(0);
                    motorRightRear.setPower(0);
                    //push jib into the stone
                    motorJib.setPower(0);
                    motorJib.setTargetPosition(jibPositionCurrent + 1000);
                    motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorJib.setPower(1); //0.5
                    flagJibHolding = true;
                    jibPositionHold = jibPositionCurrent + 1000;
                    while (motorJib.isBusy() && opModeIsActive()) {
                        //updateTelemetry();
                        if (gamepad2.b) break;
                    }
                    //activate grabber servo
                    grabberPosition = grabberClosed;
                    servoGrabber.setPosition(grabberPosition);
                    sleep(300);
                    //raise mast
                    motorMast.setPower(0);
                    motorMast.setTargetPosition(mastPositionBridgeSafe);
                    motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorMast.setPower(1); //0.3
                    flagMastHolding = true;
                    mastPositionHold =  mastPositionBridgeSafe;
                    while (motorMast.isBusy() && opModeIsActive()) {
                        if (gamepad2.b) break;
                    }
                    //pull the jib back to bumper
                    motorJib.setPower(0);
                    motorJib.setTargetPosition(jibPositionGrab + 200);
                    motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorJib.setPower(-1); // -0.5
                    flagJibHolding = true;
                    jibPositionHold = jibPositionGrab;
                }
            }
        }
    }

    private void moveDownPresetHeights() {
        for (int i = mastPresetHeights.length - 1; i >= 0; i--) {
            if (mastPositionCurrent > (mastPresetHeights[i] + 100)) {
                motorMast.setPower(0);
                motorMast.setTargetPosition(mastPresetHeights[i]);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(-0.5);

                flagMastHolding = true;
                mastPositionHold = mastPresetHeights[i];
                break;
            }
        }
    }

    private void moveUpPresetHeights() {
        for (int i = 0; i < mastPresetHeights.length; i++) {
            if (mastPositionCurrent < (mastPresetHeights[i] - 100)) {
                motorMast.setPower(0);
                motorMast.setTargetPosition(mastPresetHeights[i]);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(1);

                flagMastHolding = true;
                mastPositionHold = mastPresetHeights[i];
                break;
            }
        }
    }

    private void  moveGrabberToFoundation() {
        //position the jib
        motorJib.setPower(0);
        motorJib.setTargetPosition(jibPositionPlace);
        motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorJib.setPower(0.5);

        flagJibHolding = true;
        jibPositionHold = jibPositionPlace;
    }

    private void parkGrabber() {
        if (grabberPosition != grabberClosed) {
            //lower the mast
            motorMast.setPower(0);
            motorMast.setTargetPosition(mastPositionMin);
            motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorMast.setPower(-1);
            //position the jib
            motorJib.setPower(0);
            motorJib.setTargetPosition(jibPositionPark);
            motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorJib.setPower(-0.5);
            //ready the grabber
            grabberPosition = grabberSafe;

            flagMastHolding = true;
            flagJibHolding = true;
            mastPositionHold = mastPositionMin;
            jibPositionHold = jibPositionPark;
        }
    }

    private void readyGrabber() {
        if (grabberPosition != grabberClosed) {
            //lower the mast
            motorMast.setPower(0);
            motorMast.setTargetPosition(mastPositionMin);
            motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorMast.setPower(0.5);
            //position the jib
            motorJib.setPower(0);
            motorJib.setTargetPosition(jibPositionGrab);
            motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorJib.setPower(0.5);
            //ready the grabber
            grabberPosition = grabberOpen;

            flagMastHolding = true;
            flagJibHolding = true;
            mastPositionHold = mastPositionMin;
            jibPositionHold = jibPositionGrab;        }
    }

    private void Telemetry_LoopStart() {
        if (!flagCraneIsHomed) {
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

    private void homeCrane() {
        if (homingState == 1) timerHoming.reset();

        // do this every loop
        timeHoming = timerHoming.seconds();
        mastPositionCurrent = motorMast.getCurrentPosition();
        jibPositionCurrent = motorJib.getCurrentPosition();
        if (gamepad2.b || timeHoming > homingTimeLimit) homingState = 8001;

        //homing is cancelled
        if (homingState == 8001) {
            motorMast.setPower(0);
            motorMast.setTargetPosition(mastPositionCurrent);
            motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorMast.setPower(0.05);
            motorJib.setPower(0);
            motorJib.setTargetPosition(jibPositionCurrent);
            motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorJib.setPower(0.05);
            grabberPosition = grabberOpen;
            homingState = 0;
            flagJibHolding = true;
            flagMastHolding = true;
        }

        //initialize the routine
        if (homingState == 1) {
            motorMast.setPower(0);
            motorJib.setPower(0);
            servoGrabber.setPosition(grabberHoming);
            grabberPosition = grabberSafe;
            flagCraneIsHomed = false;
            flagMastHolding = false;
            flagJibHolding = false;
            //get ready for next state
            homingState++;
            motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorMast.setPower(0.5);
        }

        //moves mast a safe distance up so jib clears the foam
        if (homingState == 2) {
            if (digitalMastHigh.getState() || mastPositionCurrent >= mastPositionJibSafe) {
                // hold the mast at its current position
                motorMast.setPower(0);
                motorMast.setTargetPosition(mastPositionCurrent);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(0.05);
                flagMastHolding = true;
                //get ready for next state
                homingState++;
                servoGrabber.setPosition(grabberClosed);
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorJib.setPower(-0.35);
            }
        }

        //retracting jib until limit switch
        if (homingState == 3) {
            if (digitalJibLow.getState()) {
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                jibPositionCurrent = 0;
                homingDelay();
                //get ready for next state
                homingState++;
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorJib.setPower(0.1);
            }
        }

        //extend jib until limit switch is released
        if (homingState == 4) {
            if (!digitalJibLow.getState() || jibPositionCurrent > 250) {
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                jibPositionCurrent = 0;
                //get ready for the next state
                homingState++;
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorJib.setPower(1);
            }
        }

        // extend jib to safe spot
        if (homingState == 5) {
            if (jibPositionCurrent >= jibPositionPark) {
                //Hold jib in current position
                motorJib.setPower(0);
                motorJib.setTargetPosition(jibPositionCurrent);
                motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorJib.setPower(0.05);
                flagJibHolding = true;
                //get ready for next state
                homingState++;
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorMast.setPower(-0.5);
                flagMastHolding = false;
                servoGrabber.setPosition(grabberOpen);
            }
        }

        // lower mast to limit switch
        if (homingState == 6) {
            if (digitalMastLow.getState()) {
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mastPositionCurrent = 0;
                homingDelay();
                //get ready for next state
                homingState++;
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorMast.setPower(0.25);
            }
        }

        //lift mast for second pass
        if (homingState == 7) {
            if (mastPositionCurrent >= 250) {
                //get ready for next state
                homingState++;
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorMast.setPower(-0.2);
            }
        }

        //lower with precision
        if (homingState == 8) {
            if (digitalMastLow.getState()) {
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mastPositionCurrent = 0;
                //get ready for next state
                homingState++;
                motorMast.setPower(0);
                motorMast.setTargetPosition(0);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(0.05);
                flagMastHolding = true;
            }
        }

        //all done :)
        if (homingState == 9) {
            flagCraneIsHomed = true;
            servoGrabber.setPosition(grabberSafe);
            homingState = 0;
        }
    }

    private void controlServos() {
        servoGrabber.setPosition(grabberPosition);
        servoCapstone.setPosition(capstonePosition);
//        if (whiskerPosition == whiskerUp) {
//            servoLeftWhisker.setPosition(whiskerPosition);
//            servoRightWhisker.setPosition(whiskerPosition);
//        }

        //new to allow whiskers while supporting top block
        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            whiskerPosition = whiskerDown;
            servoLeftWhisker.setPosition(whiskerDown);
            servoRightWhisker.setPosition(whiskerDown);
            driveSpeedLimit = Math.min(driveSpeedLimit, whiskerSpeedLimit);
        } else {
            if (whiskerPosition == whiskerDown && Math.abs(jibPositionCurrent - jibPositionPark) < 100 && flagCraneIsHomed) {
                servoLeftWhisker.setPosition(whiskerPosition);
                servoRightWhisker.setPosition(whiskerPosition);
                driveSpeedLimit = Math.min(driveSpeedLimit, whiskerSpeedLimit);
            } else {
                servoLeftWhisker.setPosition(whiskerUp);
                servoRightWhisker.setPosition(whiskerUp);
            }
        }

    }


    private void controlMast() {
        int slowPoint = 2000;

        mastPositionCurrent = motorMast.getCurrentPosition();
        //upper limits
        if (controlMastPower > 0 && digitalMastHigh.getState()) controlMastPower = 0;
        if (controlMastPower > 0 && mastPositionCurrent > mastPositionMax) controlMastPower = 0;
        //lower limits
        if (controlMastPower < 0 && digitalMastLow.getState()) controlMastPower = 0;
        if (controlMastPower < 0 && mastPositionCurrent < mastPositionMin) controlMastPower = 0;
        //slow down
        if (controlMastPower > 0 && (mastPositionMax - mastPositionCurrent) < slowPoint) {
            controlMastPower = controlMastPower * (mastPositionMax - mastPositionCurrent) / slowPoint;
            controlMastPower = Math.max(0.05, controlMastPower);
        }
        if (controlMastPower < 0 && (mastPositionCurrent - mastPositionMin) < slowPoint) {
            controlMastPower = controlMastPower * (mastPositionCurrent - mastPositionMin) / slowPoint;
            controlMastPower = Math.min(-0.05, controlMastPower);
        }
        //hold if power is 0
        if (controlMastPower == 0) {
            if (!flagMastHolding) {
                motorMast.setPower(0);
                motorMast.setTargetPosition(mastPositionCurrent);
                motorMast.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorMast.setPower(0.05);
                flagMastHolding = true;
                mastPositionHold = mastPositionCurrent;
            }
            telemetry.addData("Mast Holding At", mastPositionHold);
        }
        //if power isn't 0 make mast go that speed
        if (controlMastPower != 0) {
            if (flagMastHolding){
                motorMast.setPower(0);
                motorMast.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flagMastHolding = false;
            }
            motorMast.setPower(controlMastPower);
            telemetry.addData("Mast Power", controlMastPower);
        }
    }

    private void controlJib() {
        int slowPoint = 2000;
        int jibPositionMinLocal = jibPositionPark;

        if (grabberPosition == grabberClosed) jibPositionMinLocal = jibPositionGrab;
        jibPositionCurrent = motorJib.getCurrentPosition();
        //upper limits
        if (controlJibPower > 0 && digitalJibHigh.getState()) controlJibPower = 0;
        if (controlJibPower > 0 && jibPositionCurrent > jibPositionMax) controlJibPower = 0;
        //lower limits
        if (controlJibPower < 0 && digitalJibLow.getState()) controlJibPower = 0;
        if (controlJibPower < 0 && jibPositionCurrent < jibPositionMinLocal) controlJibPower = 0;
        //slow down
        if (controlJibPower > 0 && (jibPositionMax - jibPositionCurrent) < slowPoint) {
            controlJibPower = controlJibPower * (jibPositionMax - jibPositionCurrent) / slowPoint;
            controlJibPower = Math.max(0.05, controlJibPower);
        }
        if (controlJibPower < 0 && (jibPositionCurrent - jibPositionMinLocal) < slowPoint) {
            controlJibPower = controlJibPower * (jibPositionCurrent - jibPositionMinLocal) / slowPoint;
            controlJibPower = Math.min(-0.05, controlJibPower);
        }
        //hold if power is 0
        if (controlJibPower == 0) {
            if (!flagJibHolding) {
                motorJib.setPower(0);
                motorJib.setTargetPosition(jibPositionCurrent);
                motorJib.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorJib.setPower(0.05);
                flagJibHolding = true;
                jibPositionHold = jibPositionCurrent;
            }
            telemetry.addData("Jib Holding At", jibPositionHold);
        }
        //if power isn't 0 make jib go that speed
        if (controlJibPower != 0) {
            if (flagJibHolding){
                motorJib.setPower(0);
                motorJib.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                flagJibHolding = false;
            }
            motorJib.setPower(controlJibPower);
            telemetry.addData("Jib Power", controlJibPower);
        }
    }


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

    //for the jib
    private void homingDelay() {
        sleep(50);
    }
}
