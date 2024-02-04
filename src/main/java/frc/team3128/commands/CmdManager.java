package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
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
        return sequence(
            rampUp(dist),
            waitUntil(climber::atSetpoint),
            waitUntil(shooter::atSetpoint),
            intake.outtake(),
            waitSeconds(1),
            neutral()
        );
    }

    public static Command shootRam() {
        return sequence(
            climber.climbTo(0.25),
            shooter.shoot(5000),
            waitUntil(()-> climber.atSetpoint()),
            waitUntil(()-> shooter.atSetpoint()),
            waitSeconds(0.5),
            intake.outtake(),
            waitSeconds(0.5),
            shooter.setShooter(0),
            intake.setRoller(0),
            climber.climbTo(0),
            waitUntil(()-> climber.atSetpoint()),
            // climber.setClimber(-0.5),
            waitSeconds(0.1),
            // climber.setClimber(0),
            waitSeconds(0.1),
            // climber.reset(),
            intake.reset()
        );
    }

    public static Command rampUp(double dist){
        return sequence(
            climber.climbTo(climber.interpolate(dist)),
            shooter.shoot(shooter.interpolate(dist))
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