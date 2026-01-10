package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gimbal extends SubsystemBase {
    private PIDController pidController;
    private DcMotor gimbalMotor;

    public Gimbal(HardwareMap hardwareMap) {

    }
}
