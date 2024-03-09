package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import common.hardware.camera.Camera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

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

    public enum ShootPosition {
        // find values
        TOP(25.0),
        BOTTOM(25.0);

        private final double height;
        ShootPosition(double height) {
            this.height = height;
        }
        public double getHeight() {
            return height;
        }
    }

    private static final Swerve swerve = Swerve.getInstance();
    private static final Climber climber = Climber.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static double vx = 0, vy = 0;
    private static boolean turning = false;

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        // TODO: add commands
        NamedCommands.registerCommand("Intake", intake.intakeAuto());
        NamedCommands.registerCommand("Shoot", autoShoot());
        NamedCommands.registerCommand("QuickShoot", quickShoot());
        NamedCommands.registerCommand("Shoot2", autoShoot2());
        NamedCommands.registerCommand("RampUpTop", rampUpAuto(ShootPosition.TOP));
        NamedCommands.registerCommand("RampUpBottom", rampUpAuto(ShootPosition.BOTTOM));
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
        else {
            vx = velocity.vxMetersPerSecond;
            vy = velocity.vyMetersPerSecond;
        }
    }

    public static Command turnInPlace() {
        DoubleSupplier setpoint = ()-> swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue);
        return new NAR_PIDCommand(
            TURN_CONTROLLER, 
            ()-> swerve.getYaw(), //measurement
            setpoint, //setpoint
            (double output) -> {
                Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
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

    public static Command quickShoot() {
        return either(
            sequence(
                waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()),
                intake.intakeRollers.outtakeNoRequirements(),
                waitSeconds(0.1),
                neutralAuto()
            ),
            none(),
            ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static Command rampUpAuto(ShootPosition pos) {
        return either(
            sequence(
                either(intake.retractAuto(), none(), ()-> intake.intakePivot.isEnabled()),
                rampUp(ShooterConstants.MAX_RPM, pos.getHeight())
            ),
            none(),
            ()->intake.intakeRollers.hasObjectPresent()
        );
    }
 
    public static Command autoShoot() {
        return either(
            sequence(
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
            ),
            none(),
            ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static Command autoShoot2() {
        return either(
            sequence(
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
            ),
            none(),
            ()->intake.intakeRollers.hasObjectPresent()
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