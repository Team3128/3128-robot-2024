package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.ShooterConstants.RAM_SHOT_RPM;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.commands.CmdSwerveDrive;

import static frc.team3128.commands.CmdManager.rampUp;

import java.util.function.DoubleSupplier;

import frc.team3128.subsystems.AmpMechanism;
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
        WING(5.75),
        TOP_PRELOAD(6.2),
        BOTTOM(4);

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
        NamedCommands.registerCommand("Shoot", autoShoot(0.75));
        NamedCommands.registerCommand("TurnShoot", autoShoot(1.25));
        NamedCommands.registerCommand("ShootSkip", either(autoShoot(0.5), none(), ()-> intake.intakeRollers.hasObjectPresent()));
        NamedCommands.registerCommand("RamShoot", ramShotAuto());
        NamedCommands.registerCommand("WingRamp", rampUpAuto(ShootPosition.WING));
        NamedCommands.registerCommand("Outtake", outtakeAuto());
        NamedCommands.registerCommand("Retract", intake.retractAuto());
        NamedCommands.registerCommand("Neutral", neutralAuto());
        NamedCommands.registerCommand("NeutralWait", sequence(neutralAuto(), waitUntil(()-> climber.atSetpoint())));

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
    public static Command ramShotAuto() {
        return sequence(
            climber.climbTo(Climber.Setpoint.RAMSHOT),
            shooter.shoot(RAM_SHOT_RPM, RAM_SHOT_RPM),
            waitUntil(()-> climber.atSetpoint() && shooter.atSetpoint()),
            intake.intakeRollers.outtakeNoRequirements(),
            waitSeconds(0.35),
            neutralAuto()
        );
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

    public static Command rampUpAuto(ShootPosition pos) {
        return either(
            sequence(
                shooter.shoot(MAX_RPM),
                either(intake.retractAuto(), none(), ()-> intake.intakePivot.isEnabled()),
                rampUp(ShooterConstants.MAX_RPM, pos.getHeight())
            ),
            none(),
            ()-> true
            // ()->intake.intakeRollers.hasObjectPresent()
        );
    }
 
    public static Command autoShoot(double turnTimeout) {
        return either(
            sequence(
                either(shooter.shoot(MAX_RPM), none(), ()-> shooter.isEnabled()),
                parallel(
                    sequence(
                        either(sequence(waitUntil(()-> intake.intakeRollers.hasObjectPresent()).withTimeout(0.25), intake.retractAuto()), none(), ()-> intake.intakePivot.isEnabled()),
                        runOnce(()->{turning = true;}),
                        rampUp()
                    ),
                    turnInPlace().withTimeout(turnTimeout)
                ),
                // waitSeconds(1),
                runOnce(()->{turning = false;}),
                intake.intakeRollers.outtake(),
                waitSeconds(0.25),
                intake.intakeRollers.runManipulator(0)
                // shooter.setShooter(0)
                // neutralAuto()
            ),
            none(),
            ()-> true
            // ()->intake.intakeRollers.hasObjectPresent()
        );
    }

    public static Command outtakeAuto() {
        return either(
            sequence(
                intake.intakeRollers.outtake(),
                waitSeconds(0.25),
                intake.intakeRollers.runManipulator(0)
            ),
            none(),
            ()-> intake.intakeRollers.hasObjectPresent()
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
            // runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
            AmpMechanism.getInstance().reset(-90),
            runOnce(()-> swerve.resetEncoders()),
            runOnce(()-> Intake.getInstance().isRetracting = false)
        );
        
    }
    
}