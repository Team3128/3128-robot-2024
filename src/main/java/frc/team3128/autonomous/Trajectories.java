package frc.team3128.autonomous;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.controllers.Controller.Type;
import common.utility.shuffleboard.NAR_Shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.IntakeConstants.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.ShooterConstants.MAX_RPM;
import static frc.team3128.Constants.ShooterConstants.RAM_SHOT_RPM;

import frc.team3128.Constants.AutoConstants;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.commands.CmdManager;
import java.util.function.DoubleSupplier;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    //USED FOR HARDCODED SHOTS
    public enum ShootPosition {
        // find values
        WING(5.213),    //Change this for top 
        BOTTOM(7.5),    //Change this for bottom auto
        RAM(24.5);

        private final double height;
        ShootPosition(double height) {
            this.height = height;
        }
        public double getHeight() {
            return height;
        }
    }

    private static double vx = 0, vy = 0;
    private static boolean turning = false;

    public static void initTrajectories() {
        Pathfinding.setPathfinder(new LocalADStar());

        // NamedCommands.registerCommand("Intake", intake.intakeAuto());
        // NamedCommands.registerCommand("Shoot", autoShoot(0.75));

    // public static Command turnDegrees(boolean counterClockwise, double angle) {
    //     DoubleSupplier setpoint = ()-> swerve.getYaw() + angle * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1);
    //     Controller controller = new Controller(new PIDFFConfig(5, 0, 0, turnkS, 0, 0), Type.POSITION);
    //     controller.enableContinuousInput(-180, 180);
    //     controller.setTolerance(1);
    //     return new NAR_PIDCommand(
    //         controller, 
    //         ()-> swerve.getYaw(), //measurement
    //         setpoint, //setpoint
    //         (double output) -> {
    //             Swerve.getInstance().drive(new ChassisSpeeds(vx, vy, Units.degreesToRadians(output)));
    //             NAR_Shuffleboard.addData("HElp", "help", output, 0, 0);
    //         }
    //     ).beforeStarting(runOnce(()-> CmdSwerveDrive.disableTurn()));
    // }

    // public static Command alignPreload(boolean counterClockwise) {
    //     return race(
    //         intake.intakeAuto(), 
    //         sequence(
    //             turnDegrees(false, 70).until(()-> RobotContainer.limelight.hasValidTarget()),
    //             repeatingSequence(
    //                 runOnce(()-> CmdAutoAlign.hasTimedOut = false),
    //                 new CmdAutoAlign(3, false),
    //                 run(()-> swerve.drive(
    //                     new Translation2d(), 
    //                     maxAngularVelocity / 4.0 * (counterClockwise ? 1 : -1) * (Robot.getAlliance() == Alliance.Red ? -1 : 1), 
    //                     false)
    //                 ).until(()-> RobotContainer.limelight.hasValidTarget())
    //             ).until(()-> intake.intakeRollers.hasObjectPresent())
    //         ).beforeStarting(runOnce(()->{turning = true;})).andThen(runOnce(()->{turning = false;})));
    // }



    // public static double getDistance(double d, double e) {
    //     return Math.sqrt((d*d)+((5.4-e)*(5.4-e)));
    //     //gets distance of point based on blue's focal point (0, 5.4)
    // }

 
    // }

    // public static Command neutralAuto() {
      // return sequence(
        //     intake.intakeRollers.runNoRequirements(0),
        //     // Shooter.getInstance().setShooter(0),
        //     Climber.getInstance().climbTo(Climber.Setpoint.RETRACTED)
        // );
    // }

    // public static Command getPathPlannerAuto(String name) {
    //     return new PathPlannerAuto(name);
    // }

    // public static Command resetAuto() {
        // return sequence(
        //     intake.intakePivot.reset(0),
        //     climber.reset(),
        //     runOnce(()-> swerve.zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180)),
        //     AmpMechanism.getInstance().reset(-90),
        //     runOnce(()-> swerve.resetEncoders()),
        //     runOnce(()-> Intake.getInstance().isRetracting = false)
        // );
        
    // }

    // public static Command goToPoint(Pose2d pose) {
        // return AutoBuilder.pathfindToPose(
        //         pose,
        //         AutoConstants.constraints,
        //         0.0, // Goal end velocity in meters/sec
        //         0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        //     );
    //}
    
}
}