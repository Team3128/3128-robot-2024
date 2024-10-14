package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.subsystems.Climber;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.*;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

public class CmdManager {


    //private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static boolean climb = false;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    // public static Command shootDist() {
    //     return deadline(
    //         sequence(
    //             climber.climbTo(()-> climber.interpolate(swerve.getDist())),
    //             shooter.shoot(MAX_RPM),
    //             waitUntil(shooter::atSetpoint),
    //             climber.climbTo(() -> climber.interpolate(swerve.getDist())),
    //             waitUntil(climber::atSetpoint),
    //             intake.intakeRollers.outtakeWithTimeout(0.35),
    //             neutral()
    //         ),
    //         repeatingSequence(  
    //             runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
    //             waitSeconds(0.1)
    //         )
    //     ).andThen(runOnce(()-> CmdSwerveDrive.disableTurn()));
    // }
    //}
}