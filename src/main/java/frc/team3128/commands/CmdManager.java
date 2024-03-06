package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import common.utility.Log;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command autoShoot() {
        return sequence(
            // runOnce(()-> DriverStation.reportWarning("AutoShoot: CommandStarting", false)),
            parallel(
                rampUp(),
                swerve.turnInPlace(true).asProxy().withTimeout(1)
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false)
            // runOnce(()-> DriverStation.reportWarning("AutoShoot: CommandEnding", false))
        );
    }

    public static Command ampShootAlt() {
        return sequence(
            intake.intakePivot.pivotNoRequirements(-87),
            waitUntil(()-> intake.intakePivot.atSetpoint()),
            intake.intakeRollers.runManipulator(-0.2875),
            waitSeconds(0.2),
            intake.retract(false)
        );
    }

    public static Command ampShoot() {
        return sequence (
            intake.intakePivot.pivotNoRequirements(-72.5),
            waitUntil(()-> intake.intakePivot.getMeasurement() < -55),
            intake.intakeRollers.runNoRequirements(IntakeConstants.AMP_POWER),
            waitSeconds(0.25),
            intake.retract(false)
        );
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            runOnce(()-> DriverStation.reportWarning("Shoot: CommandStarting", false)),
            rampUp(rpm, height),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false),
            runOnce(()-> DriverStation.reportWarning("Shoot: CommandEnding", false))
        );
    }

    public static Command feed(double rpm, double height){
        return sequence(
            parallel(
                swerve.turnInPlace(()-> Robot.getAlliance() == Alliance.Blue ? 155 : 35).asProxy().withTimeout(1),
                rampUp(rpm, height)
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false)
        );
    }

    public static Command rampUp() {
        return rampUp(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getPredictedDistance()));
    }

    public static Command rampUp(double rpm, double height){
        return rampUp(rpm, ()-> height);
    }

    public static Command rampUp(double rpm, DoubleSupplier height) {
        return sequence(
            climber.climbTo(height),
            shooter.shoot(rpm),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint())
        );
    }

    public static Command rampUpAmp() {
        return sequence(
            climber.climbTo(Climber.Setpoint.AMP),
            shooter.runBottomRollers(ShooterConstants.AMP_POWER)
        );
    }

    public static Command rampUpContinuous() {
        return rampUpContinuous(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getPredictedDistance()));
    }

    public static Command rampUpContinuous(double rpm, DoubleSupplier height) {
        return sequence(
            shooter.shoot(rpm),
            repeatingSequence(
                climber.climbTo(height),
                waitSeconds(0.1)
            )
        );
    }

    public static Command neutral(boolean shouldStall){
        return sequence(
            runOnce(()-> DriverStation.reportWarning("Neutral: CommandStarting", false)),
            intake.intakeRollers.runNoRequirements(0),
            shooter.setShooter(0),
            climber.climbTo(Climber.Setpoint.RETRACTED),
            waitUntil(()-> climber.atSetpoint()),
            climber.setClimber(-0.5),
            waitSeconds(0.1),
            climber.setClimber(0),
            parallel(
                // intake.retract(shouldStall),
                sequence(
                    waitSeconds(0.5),
                    climber.reset()
                )
            ),
            runOnce(()-> DriverStation.reportWarning("Neutral: CommandEnding", false))
        );
    }
}