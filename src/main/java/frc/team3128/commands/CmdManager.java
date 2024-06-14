package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.subsystems.AmpMechanism;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Setpoint;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();
    private static AmpMechanism ampMechanism = AmpMechanism.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command shootDist() {
        return deadline(
            sequence(
                climber.climbTo(()->climber.interpolate(swerve.getDist())),
                shooter.shoot(MAX_RPM),
                waitUntil(shooter::atSetpoint),
                climber.climbTo(()->climber.interpolate(swerve.getDist())),
                waitUntil(climber::atSetpoint),
                intake.intakeRollers.outtakeWithTimeout(0.35),
                neutral()
            ),
            repeatingSequence(  
                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                waitSeconds(0.1)
            )
        ).andThen(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command autoShoot() {
        return sequence(
            parallel(
                rampUp().withTimeout(RAMP_TIME),
                swerve.turnInPlace(false).asProxy().withTimeout(1)
            ),
            intake.intakeRollers.outtakeWithTimeout(OUTTAKE_TIMEOUT),
            neutral()
        );
    }

    public static Command ampShoot() {
        return sequence (
            rampUp(()->Climber.Setpoint.AMP.setpoint, AMP_RPM),
            ampMechanism.extend(),
            waitUntil(()-> ampMechanism.atSetpoint() && shooter.atSetpoint()),
            intake.intakeRollers.ampOuttake(0.9),
            ampMechanism.retract(),
            neutral()
        );
    }

    public static Command ramShot() {
        return sequence(
            rampUp(()->Climber.Setpoint.RAMSHOT.setpoint, RAM_SHOT_RPM).withTimeout(1.5),
            intake.intakeRollers.outtakeWithTimeout(OUTTAKE_TIMEOUT),
            neutral()
        );
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            rampUp(()->rpm, height).withTimeout(RAMP_TIME),
            intake.intakeRollers.outtakeWithTimeout(OUTTAKE_TIMEOUT),
            neutral()
        );
    }

    public static Command feed(double rpm, double height, double angle){
        return sequence(
            parallel(
                swerve.turnInPlace(()-> Robot.getAlliance() == Alliance.Blue ? 180-angle : angle).asProxy().withTimeout(1),
                rampUp(()->rpm, height),
                runOnce(()-> shooter.startPID(rpm, rpm))
            ),
            intake.intakeRollers.outtakeWithTimeout(OUTTAKE_TIMEOUT),
            neutral()
        );
    }

    public static Command rampUp() {
        return rampUp(()->climber.interpolate(swerve.getDist()), ShooterConstants.MAX_RPM);
    }

    public static Command rampUp(DoubleSupplier height, double... rpm) {
        double left = rpm[0];
        double right = rpm.length > 1 ? rpm[1] : left; 
        return sequence(
            shooter.shoot(left, right),
            climber.climbTo(height),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint())
        );
    }

    public static Command neutral(){
        return sequence(
            shooter.setShooter(0),
            climber.climbTo(Climber.Setpoint.RETRACTED),
            waitUntil(()-> climber.atSetpoint()),
            climber.hardReset()
        );
    }

}