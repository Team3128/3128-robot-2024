// package frc.team3128.autonomous;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import common.core.commands.NAR_PIDCommand;
// import common.core.controllers.Controller;
// import common.hardware.limelight.Limelight;
// import common.core.controllers.Controller.Type;
// import common.utility.shuffleboard.NAR_Shuffleboard;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.pathfinding.LocalADStar;
// import com.pathplanner.lib.pathfinding.Pathfinding;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import static edu.wpi.first.wpilibj2.command.Commands.*;
// import static frc.team3128.Constants.AutoConstants.*;
// import static frc.team3128.Constants.SwerveConstants.*;
// import frc.team3128.Constants.AutoConstants;
// import frc.team3128.Constants.SwerveConstants;
// import frc.team3128.Robot;
// import frc.team3128.RobotContainer;
// // import frc.team3128.commands.NAR_PIDCommand;

// import java.util.function.DoubleSupplier;

// import frc.team3128.subsystems.Swerve;

// /**
//  * Store trajectories for autonomous. Edit points here. 
//  * @author Daniel Wang
//  */
// public class Trajectories {

//     private static final Swerve swerve = Swerve.getInstance();
//     private static final Limelight limelight = RobotContainer.limelight;
//     private static double vx = 0, vy = 0;
//     private static boolean turning = false;
//     // private static final AutoPrograms autoPrograms = new AutoPr

//     public static void initTrajectories() {
//         Pathfinding.setPathfinder(new LocalADStar());

//         AutoBuilder.configureHolonomic(
//             swerve::getPose,
//             swerve::resetOdometry,
//             swerve::getRobotVelocity,
//             Swerve.getInstance()::driveOverridable,
//             new HolonomicPathFollowerConfig(
//                 new PIDConstants(translationKP, translationKI, translationKD),
//                 new PIDConstants(rotationKP, rotationKI, rotationKD),
//                 maxAttainableSpeed,
//                 trackWidth,
//                 new ReplanningConfig(false, true)
//             ),
//             ()-> Robot.getAlliance() == Alliance.Red,
//             swerve
//         );
//     }

//     public static Command getPathPlannerAuto(String name) {
//         return new PathPlannerAuto(name);
//     }

//     public static Command getPathPlannerPath(String name) {
//         return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
//     }

//     public static Command goToPoint(Pose2d pose) {
//         return AutoBuilder.pathfindToPose(
//                 pose,
//                 AutoConstants.constraints,
//                 0.0, // Goal end velocity in meters/sec
//                 0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
//             );
//     }
// }