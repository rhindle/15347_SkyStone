package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Old: Blue / Foundation / Far", group = "")
public class Autonomous_Override_Blue_Foundation_Far extends Emmet_Autonomous_Needham_Updated {

    @Override
    void setAutonomousVariables() {
        //1 = blue, 2 = red
        autoAlliance = 1;
        // 1 is foundation, 2 is quarry
        autoSide = 1;
        //1 is near, 2 is far
        autoParkingPosition = 2;
    }
}
