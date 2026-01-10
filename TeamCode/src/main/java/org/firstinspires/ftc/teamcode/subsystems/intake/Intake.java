package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private boolean isRunning;

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.isRunning = false;
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        if (isRunning) {
            intakeMotor.setPower(IntakeConstants.intakePower);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
