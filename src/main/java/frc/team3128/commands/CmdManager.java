package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;

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
        return startEnd(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(0.5);
    }

    public static Command autoShoot() {
        return sequence(
            parallel(
                rampUp(),
                swerve.turnInPlace()
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            intake.outtakeNoRequirements(),
            waitSeconds(1),
            neutral()
        );
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            rampUp(rpm, height),
            intake.outtakeNoRequirements(),
            waitSeconds(1),
            neutral()
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
            waitUntil(climber::atSetpoint),
            waitUntil(shooter::atSetpoint)
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

    public static Command neutral(){
        return sequence(
            vibrateController(),
            intake.stopRollersNoRequirements(),
            shooter.setShooter(0),
            climber.climbTo(Climber.State.RETRACTED),
            waitUntil(()-> climber.atSetpoint()),
            climber.setClimber(-0.5),
            waitSeconds(0.1),
            climber.setClimber(0),
            parallel(
                intake.retract(),
                sequence(
                    waitSeconds(0.5),
                    climber.reset()
                )
            )
        );
    }
}