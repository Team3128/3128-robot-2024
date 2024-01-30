package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.FieldConstants.*;


public class CmdManager {
    

    private Swerve swerve = Swerve.getInstance();
    private Intake intake = Intake.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Climber climber = Climber.getInstance();

    private NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public Command vibrateController(){
        return Commands.waitSeconds(0.5).deadlineWith(Commands.startEnd(()-> controller.startVibrate(), ()-> controller.stopVibrate()));
    }

    public Command pickup(){
        return Commands.parallel(
            climber.moveTo(Climber.State.RETRACTED),
            intake.intake()
        );
    }

    public Command pickupHP(){
        return Commands.parallel(
            climber.moveTo(Climber.State.RETRACTED),
            intake.intakeHP()  
        );
    }

    public Command outake(){
        return Commands.parallel(
            climber.moveTo(Climber.State.RETRACTED),
            intake.outake()
        );
    }

    public Command shoot(){ 
        double dist = swerve.getPose().relativeTo(SPEAKER).getTranslation().getNorm();
        return Commands.sequence(
            intake.retract(),
            rampUp(dist),
            intake.shoot(),
            Commands.waitSeconds(0.2),
            neutral()
        );
    }

    public Command rampUp(double dist){
        return Commands.parallel(
            climber.shoot(dist),
            shooter.shoot(dist)
        );
    }

    public Command climb(){
        return Commands.sequence(
            Commands.runOnce(()-> climb = false),
            climber.moveTo(Climber.State.EXTENDED),
            Commands.waitUntil(() -> climb),
            climber.moveTo(Climber.State.RETRACTED)
            // maybe shoot trap
        );
    }

    public Command neutral(){
        return Commands.sequence(
            Commands.runOnce(()-> shooter.stop(), shooter),
            climber.moveTo(Climber.State.RETRACTED),
            intake.retract()
        );
    }

    public Command fullStop(){
        return Commands.parallel(
            Commands.runOnce(()-> shooter.stop(), shooter),
            Commands.runOnce(()-> swerve.stop(), swerve),
            Commands.runOnce(()-> intake.stop(), intake),
            Commands.runOnce(()-> climber.stop(), climber)
        );
    }

    public Command fullReset(){
        return Commands.parallel(
            Commands.runOnce(()-> intake.reset(), intake),
            Commands.runOnce(()-> climber.reset(), climber)
        );
    }
}