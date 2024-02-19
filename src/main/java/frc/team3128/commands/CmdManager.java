package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.AMP_POWER;


public class CmdManager {
    

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public static Command vibrateController(){
        return startEnd(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(0.5);
    }

    public static Command autoShoot() {
        return sequence(
            parallel(
                rampUp(),
                swerve.turnInPlace().asProxy()
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false)
        );
    }

    public static Command ampShoot() {
        return sequence (
            shooter.runBottomRollers(AMP_POWER),
            climber.climbTo(Climber.State.AMP),
            waitUntil(() -> climber.atSetpoint()),
            intake.intakeRollers.outtake(),
            waitSeconds(0.1),
            neutral(false)
        );
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            rampUp(rpm, height),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false)
        );
    }

    public static Command feed(double rpm, double height, double angle){
        return sequence(
            runOnce(()-> {
                CmdSwerveDrive.setTurnSetpoint(angle);
            }),
            rampUp(rpm, height),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutral(false)
        );
    }

    public static Command rampUp() {
        return rampUp(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getDist()));
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
            climber.climbTo(Climber.State.AMP),
            shooter.runBottomRollers(AMP_POWER)
        );
    }

    public static Command rampUpContinuous() {
        return rampUpContinuous(ShooterConstants.MAX_RPM, ()-> climber.interpolate(swerve.getDist()));
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
            vibrateController(),
            intake.intakeRollers.runNoRequirements(0),
            shooter.setShooter(0),
            climber.climbTo(Climber.State.RETRACTED),
            waitUntil(()-> climber.atSetpoint()),
            climber.setClimber(-0.5),
            waitSeconds(0.1),
            climber.setClimber(0),
            parallel(
                intake.retract(shouldStall),
                sequence(
                    waitSeconds(0.5),
                    climber.reset()
                )
            )
        );
    }
}