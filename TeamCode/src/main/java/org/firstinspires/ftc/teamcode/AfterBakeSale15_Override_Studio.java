package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Experiment_15_Override (Studio)", group = "")
@Disabled
public class AfterBakeSale15_Override_Studio extends AfterBakeSale15_HomingTime_Studio {

    // todo: write your code here

    @Override
    void setDefaults() {
        Default_Message = "Experiment_15_Override_Successful!";
    }
}
