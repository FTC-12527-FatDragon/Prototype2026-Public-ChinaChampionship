package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

import java.util.concurrent.TimeUnit;

@Config
@Configurable
public abstract class TeleOpBase extends CommandOpMode {
    public Drive drive;
    public GamepadEx gamepadEx1;
    public Shooter shooter;
    public Transit transit;
    public Intake intake;
    public ElapsedTime timer;

    protected abstract Drive.Alliance getAlliance();

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap, getAlliance());
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap, false);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
        timer = new ElapsedTime();

        timer.reset();

        new FunctionalButton(
                () -> timer.time(TimeUnit.SECONDS) == 90
        ).whenPressed(
                new InstantCommand(() -> gamepad1.rumble(1.0, 1.0, 500))
        );

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1,
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)));


        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.B)
        ).whenPressed(
                new InstantCommand(() -> drive.visionCalibrate())
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> drive.reset(0))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenHeld(
                new IntakeCommand(transit, intake)
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new TransitCommand()
                        .alongWith(new InstantCommand(() -> drive.visionCalibrate()))
        );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("VisionPose", drive.getVisionPose() != null);
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
        telemetry.addData("ShooterVelocity", shooter.shooterState.toString());
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        telemetry.addData("LF Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", drive.rightBackMotor.getPower());
        telemetry.addData("LF Mode: ", drive.leftFrontMotor.getMode());
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", shooter.getVelocity());
        packet.put("StopTime", transit.stopTime);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
