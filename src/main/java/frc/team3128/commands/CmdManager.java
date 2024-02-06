package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
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
        return parallel(
            shoot(),
            swerve.CmdTurnInPlace(()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))
        );
    }

    public static Command shoot(){ 
        return shoot(5700, climber.interpolate(swerve.getDist(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue)));
    }

    public static Command shoot(double rpm, double height){
        return sequence(
            rampUp(rpm, height),
            intake.outtake(),
            waitSeconds(1),
            neutral()
        );
    }

    public static Command rampUp(double speed, double height){
        return sequence(
            climber.climbTo(height),
            shooter.shoot(speed),
            waitUntil(climber::atSetpoint),
            waitUntil(shooter::atSetpoint)
        );
    }

    public static Command shootRam() {
        return shoot(5000, 0.25);
    }

    public static Command outtakeRetract(){
        return sequence(
            intake.outtake(),
            waitSeconds(1),
            neutral()
        );
    }

    public static Command neutral(){
        return sequence(
            intake.setRoller(0),
            shooter.setShooter(0),
            climber.climbTo(Climber.State.RETRACTED),
            waitUntil(()-> climber.atSetpoint()),
            climber.setClimber(-0.5),
            waitSeconds(0.1),
            climber.setClimber(0),
            waitSeconds(0.5),
            climber.reset()
        );
    }

    public static Command fullReset(){
        return sequence(
            runOnce(()-> intake.reset()),
            runOnce(()-> climber.reset())
        );
    }
}