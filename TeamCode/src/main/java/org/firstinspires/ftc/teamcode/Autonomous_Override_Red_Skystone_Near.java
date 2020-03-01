package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red / Skystone / Near", group = "")
public class Autonomous_Override_Red_Skystone_Near extends Emmet_Autonomous_Needham_Updated {

    @Override
    void setAutonomousVariables() {
        //1 = blue, 2 = red
        autoAlliance = 2;
        // 1 is foundation, 2 is quarry
        autoSide = 2;
        //1 is near, 2 is far
        autoParkingPosition = 1;
    }
}
