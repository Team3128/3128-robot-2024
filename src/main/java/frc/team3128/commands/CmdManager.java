package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.IntakeConstants;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.FieldConstants.*;
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


    public static Command shoot(){ 
        double dist = swerve.getPose().relativeTo(SPEAKER).getTranslation().getNorm();
        return shoot(shooter.interpolate(dist), climber.interpolate(dist));
    }

    public static Command shoot(double speed, double height){
        return sequence(
            rampUp(speed, height),
            intake.outtake(),
            waitSeconds(1),
            neutral()
        );
    }

    public static Command rampUp(){
        double dist = swerve.getPose().relativeTo(SPEAKER).getTranslation().getNorm();
        return rampUp(shooter.interpolate(dist), climber.interpolate(dist));
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
            intake.retract()
        );
    }

    public static Command fullReset(){
        return sequence(
            runOnce(()-> intake.reset()),
            runOnce(()-> climber.reset())
        );
    }
}