package org.firstinspires.ftc.teamcode.opmodes.teleops;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

@TeleOp(name = "TeleOp Solo Red", group = "TeleOp")
public class Red extends TeleOpBase {
    @Override
    public Drive.Alliance getAlliance() {
        return Drive.Alliance.RED;
    }
}
