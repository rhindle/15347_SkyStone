

package Boneyard;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;

@TeleOp(name = "Blocky_Vuforia (BTJ_Studio)", group = "")
@Disabled
public class Blocky_Vuforia_Studio extends LinearOpMode {


    private VuforiaSkyStone vuforiaSkyStone;

    private VuforiaBase.TrackingResults vuforiaResults;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        vuforiaSkyStone = new VuforiaSkyStone();

        // Initialize Vuforia
        telemetry.addData("Status", "Initializing Vuforia. Please wait...");
        telemetry.update();
        initVuforia();
        // Activate here for camera preview.
        vuforiaSkyStone.activate();
        telemetry.addData(">>", "Vuforia initialized, press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Are the targets visible?
                // (Note we only process first visible target).
                if (isTargetVisible("Stone Target")) {
                    processTarget();
                } else {
                    telemetry.addData("No Skystone Detected", "Targets are not visible.");
                }
                telemetry.update();
            }
        }
        // Don't forget to deactivate Vuforia.
        vuforiaSkyStone.deactivate();

        vuforiaSkyStone.close();
    }

    /**
     * Describe this function...
     */
    private void OldStuff() {
        if (isTargetVisible("Stone Target")) {
            processTarget();
        } else if (isTargetVisible("Blue Front Bridge")) {
            processTarget();
        } else if (isTargetVisible("Red Rear Bridge")) {
            processTarget();
        } else if (isTargetVisible("Red Front Bridge")) {
            processTarget();
        } else if (isTargetVisible("Red Rear Bridge")) {
            processTarget();
        } else if (isTargetVisible("Red Perimeter 1")) {
            processTarget();
        } else if (isTargetVisible("Red Perimeter 2")) {
            processTarget();
        } else if (isTargetVisible("Front Perimeter 1")) {
            processTarget();
        } else if (isTargetVisible("Front Perimeter 2")) {
            processTarget();
        } else if (isTargetVisible("Blue Perimeter 1")) {
            processTarget();
        } else if (isTargetVisible("Blue Perimeter 2")) {
            processTarget();
        } else if (isTargetVisible("Rear Perimeter 1")) {
            processTarget();
        } else if (isTargetVisible("Rear Perimeter 2")) {
            processTarget();
        } else {
            telemetry.addData("No Targets Detected", "Targets are not visible.");
        }
    }

    /**
     * Describe this function...
     */
    private void initVuforia() {
        // Rotate phone -90 so back camera faces "forward" direction on robot.
        // Assumes landscape orientation
        vuforiaSkyStone.initialize(
                "", // vuforiaLicenseKey
                VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
                true, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                -90, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
    }

    /**
     * Check to see if the target is visible.
     */
    private boolean isTargetVisible(String trackableName) {
        boolean isVisible;

        // Get vuforia results for target.
        vuforiaResults = vuforiaSkyStone.track(trackableName);
        // Is this target visible?
        if (vuforiaResults.isVisible) {
            isVisible = true;
        } else {
            isVisible = false;
        }
        return isVisible;
    }

    /**
     * This function displays location on the field and rotation about the Z
     * axis on the field. It uses results from the isTargetVisible function.
     */
    private void processTarget() {
        // Display the target name.
        telemetry.addData("Target Detected", vuforiaResults.name + " is visible.");
        telemetry.addData("X (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.x, "IN"), 2)));
        telemetry.addData("Y (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.y, "IN"), 2)));
        telemetry.addData("Z (in)", Double.parseDouble(JavaUtil.formatNumber(displayValue(vuforiaResults.z, "IN"), 2)));
        telemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(vuforiaResults.zAngle, 2)));
    }

    /**
     * By default, distances are returned in millimeters by Vuforia.
     * Convert to other distance units (CM, M, IN, and FT).
     */
    private double displayValue(float originalValue, String units) {
        double convertedValue;

        // Vuforia returns distances in mm.
        if (units.equals("CM")) {
            convertedValue = originalValue / 10;
        } else if (units.equals("M")) {
            convertedValue = originalValue / 1000;
        } else if (units.equals("IN")) {
            convertedValue = originalValue / 25.4;
        } else if (units.equals("FT")) {
            convertedValue = (originalValue / 25.4) / 12;
        } else {
            convertedValue = originalValue;
        }
        return convertedValue;
    }
}

