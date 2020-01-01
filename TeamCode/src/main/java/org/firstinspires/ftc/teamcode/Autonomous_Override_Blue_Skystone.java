package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Blue / Skystone", group = "")
public class Autonomous_Override_Blue_Skystone extends Emmet_Autonomous {

    @Override
    void setAutonomousVariables() {
        //1 = blue, 2 = red
        autoAlliance = 1;
        // 1 is foundation, 2 is quarry
        autoSide = 2;
        //1 is near, 2 is far
        autoParkingPosition = 1;
    }
}
