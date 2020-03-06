package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "A--- Auto: Red 2 Stone", group = "")
public class Autonomous_Override_Red_2Stone extends Emmet_Autonomous_Natick {

    @Override
    void setAutonomousVariables() {
        //1 = blue, 2 = red
        autoAlliance = 2;
        // 1 is foundation, 2 is quarry
        autoSide = 3;
        //1 is near, 2 is far
        autoParkingPosition = 2;
    }
}
