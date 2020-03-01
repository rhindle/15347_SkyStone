package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Red / Foundation / Near", group = "")
public class Autonomous_Override_Red_Foundation_Near extends Emmet_Autonomous_Needham_Updated {

    @Override
    void setAutonomousVariables() {
        //1 = blue, 2 = red
        autoAlliance = 2;
        // 1 is foundation, 2 is quarry
        autoSide = 1;
        //1 is near, 2 is far
        autoParkingPosition = 1;
    }
}
