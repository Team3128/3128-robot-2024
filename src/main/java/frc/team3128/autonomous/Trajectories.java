package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import common.hardware.camera.Camera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.commands.CmdManager.*;

import java.util.function.DoubleSupplier;


import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
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
        NamedCommands.registerCommand("Intake", intake.intakeAuto());
        NamedCommands.registerCommand("Shoot", autoShoot());
        NamedCommands.registerCommand("ShootFast", autoShootNoTurn());
        NamedCommands.registerCommand("Shoot2", autoShoot2());
        NamedCommands.registerCommand("RampUp", rampUpAuto());
        NamedCommands.registerCommand("Retract", intake.retractAuto());
        NamedCommands.registerCommand("Amp", null);
        NamedCommands.registerCommand("Drop", null);
        NamedCommands.registerCommand("Disable", runOnce(()-> Camera.disableAll()));
        NamedCommands.registerCommand("Enable", runOnce(()-> Camera.enableAll()));

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
                Swerve.getInstance().drive(new Translation2d(), Units.degreesToRadians(output), true);
            }
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command turnInPlace2() {
        DoubleSupplier setpoint = ()-> Robot.getAlliance() == Alliance.Red ? 305 - 10 : -125 + 10;
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new Translation2d(), Units.degreesToRadians(output), true);
            }
        ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    }

    public static Command rampUpAuto() {
        return sequence(
            either(intake.retractAuto(), none(), ()-> intake.intakePivot.isEnabled()),
            rampUp(ShooterConstants.MAX_RPM, 25)
        );
    }

    public static Command autoShootNoTurn() {
        return sequence(
            rampUpAuto(),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutralAuto()
        );
    }
 
    public static Command autoShoot() {
        return sequence(
            either(intake.retractAuto(), none(), ()-> intake.intakePivot.isEnabled()),
            runOnce(()->{turning = true;}),
            parallel(
                rampUp(),
                turnInPlace().withTimeout(0.5)
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            // waitSeconds(1),
            runOnce(()->{turning = false;}),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutralAuto()
        );
    }

    public static Command autoShoot2() {
        return sequence(
            runOnce(()-> turning = true),
            parallel(
                rampUp(MAX_RPM, 12.94),
                turnInPlace2().withTimeout(0.75)
                // runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                // waitUntil(()-> CmdSwerveDrive.rController.atSetpoint())
            ),
            // waitSeconds(1),
            runOnce(()->{turning = false;}),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.1),
            neutralAuto()
        );
    }

    public static Command neutralAuto() {
        return sequence(
            intake.intakeRollers.runNoRequirements(0),
            Shooter.getInstance().setShooter(0),
            Climber.getInstance().climbTo(Climber.Setpoint.RETRACTED)
        );
    }

    public static Command getPathPlannerAuto(String name) {
        return new PathPlannerAuto(name);
    }

    // TODO: make new
    public static Command resetAuto() {
        return sequence(
            Intake.getInstance().intakePivot.reset(0),
            Climber.getInstance().reset(),
            runOnce(()-> swerve.resetEncoders()),
            runOnce(()-> Intake.getInstance().isRetracting = false)
        );
        
    }
    
}