package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Robot;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.commands.CmdManager.*;

import java.util.function.DoubleSupplier;


import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final Swerve swerve = Swerve.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static boolean turning = false;

    public static void initTrajectories() {

        // TODO: add commands
        NamedCommands.registerCommand("Intake", Intake.getInstance().intake(Intake.State.EXTENDED));
        NamedCommands.registerCommand("Shoot", autoShoot());
        NamedCommands.registerCommand("Amp", null);
        NamedCommands.registerCommand("Drop", null);

        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            Trajectories::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(translationKP, translationKI, translationKD),
                new PIDConstants(rotationKP, rotationKI, rotationKD),
                maxAttainableSpeed,
                trackWidth,
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
        );
    }

    public static void drive(ChassisSpeeds velocity) {
        if (!turning) swerve.drive(velocity);
    }

    public static Command turnInPlace() {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                // final double x = RobotContainer.controller.getLeftX();
                // final double y = RobotContainer.controller.getLeftY();
                // Translation2d translation = new Translation2d(x,y).times(maxAttainableSpeed);
                // if (Robot.getAlliance() == Alliance.Red || !swerve.fieldRelative) {
                //     translation = translation.rotateBy(Rotation2d.fromDegrees(90));
                // }
                // else {
                //     translation = translation.rotateBy(Rotation2d.fromDegrees(-90));
                // }

                Swerve.getInstance().drive(new Translation2d(), Units.degreesToRadians(output), true);
            }
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }
 
    public static Command autoShoot() {
        return sequence(
            runOnce(()->{turning = true;}),
            parallel(
                rampUp(),
                turnInPlace()
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            runOnce(()->{turning = false;}),
            intake.outtakeNoRequirements(),
            waitSeconds(1),
            neutral(false)
        );
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
    }

    // TODO: make new
    public static Command resetAuto() {
        return sequence(
            Intake.getInstance().reset(),
            Climber.getInstance().reset()
        );
        
    }
    
}