package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public Motor leftShooter;
    public Motor rightShooter;
    public TelemetryPacket packet = new TelemetryPacket();

    public ShooterState shooterState = ShooterState.STOP;

    public boolean highSpeed;

    public double dynamicSpeed;

    public Shooter(final HardwareMap hardwareMap) {
        leftShooter = new Motor(hardwareMap, ShooterConstants.leftShooterName);
        rightShooter = new Motor(hardwareMap, ShooterConstants.rightShooterName);

    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity),
        SLOW(ShooterConstants.slowVelocity),
        FAST(ShooterConstants.fastVelocity),
        DYNAMIC(0),
        OPENLOOP(0);

        final double shooterVelocity;

        ShooterState(double shooterVelocity) {
            this.shooterVelocity = shooterVelocity;
        }
    }

    public void setShooterState(ShooterState state) {
        shooterState = state;
    }

    public void setDynamicSpeed(double dynamicSpeed) {
        this.dynamicSpeed = dynamicSpeed;
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                rightShooter.getVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
//        if (shooterState != ShooterState.STOP) {
//            double currentPower = pidController.calculate(
//                    rightShooter.getVelocity(), shooterState.shooterVelocity);
//            leftShooter.setPower(-currentPower);
//            rightShooter.setPower(currentPower);
//            packet.put("currentPower", currentPower);
//        }
        if (shooterState != ShooterState.STOP) {
            releaseShooter();
            if (shooterState == ShooterState.FAST) {
                leftShooter.setPower(-ShooterConstants.fastPower);
                rightShooter.setPower(ShooterConstants.fastPower);
            }
            else if (shooterState == ShooterState.SLOW) {
                leftShooter.setPower(-ShooterConstants.slowPower);
                rightShooter.setPower(ShooterConstants.slowPower);
            }
            else if (shooterState == ShooterState.DYNAMIC){
                leftShooter.setPower(-dynamicSpeed);
                rightShooter.setPower(dynamicSpeed);
            }
        }
        else {
            if (highSpeed && getVelocity() > ShooterState.FAST.shooterVelocity) {
                brakeShooter();
            }
            else if (!highSpeed && getVelocity() > ShooterState.SLOW.shooterVelocity) {
                brakeShooter();
            }
            else releaseShooter();
            if (!highSpeed) {
                leftShooter.setPower(-ShooterState.STOP.shooterVelocity);
                rightShooter.setPower(ShooterState.STOP.shooterVelocity);
            }
            else {
                leftShooter.setPower(-ShooterState.FASTSTOP.shooterVelocity);
                rightShooter.setPower(ShooterState.FASTSTOP.shooterVelocity);
            }
        }

        packet.put("shooterVelocity", rightShooter.getVelocity());
        packet.put("shooterPower", rightShooter.getPower());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
