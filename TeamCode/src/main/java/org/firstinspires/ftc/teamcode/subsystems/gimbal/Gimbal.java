package org.firstinspires.ftc.teamcode.subsystems.gimbal;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gimbal extends SubsystemBase {
    private PIDController pidController;
    private DcMotor gimbalMotor;
    public enum GimbalState {
        INIT,
        ACTIVE;

        GimbalState() {}
    }
    private GimbalState gimbalState;

    public Gimbal(HardwareMap hardwareMap) {
        this.gimbalMotor = hardwareMap.get(DcMotor.class, GimbalConstants.gimbalMotorName);
        this.pidController = new PIDController(
                GimbalConstants.kp, GimbalConstants.ki, GimbalConstants.kd
        );
        this.gimbalState = GimbalState.INIT;
        this.pidController.setSetPoint(GimbalConstants.initPos);
    }

    @Override
    public void periodic() {
        if (!pidController.atSetPoint()) {
            gimbalMotor.setPower(pidController.calculate(gimbalMotor.getCurrentPosition()));
        }

    }
}
