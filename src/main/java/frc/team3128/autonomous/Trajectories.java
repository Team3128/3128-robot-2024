package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.AutoConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final HashMap<String, List<PathPlannerTrajectory>> trajectories = new HashMap<String, List<PathPlannerTrajectory>>();

    private static final HashMap<String, Command> CommandEventMap = new HashMap<String, Command>();

    private static final Swerve swerve = Swerve.getInstance();

    private static SwerveAutoBuilder builder;

    public static void initTrajectories() {
        final String[] trajectoryNames = {};

        for (final String trajectoryName : trajectoryNames) {

            if (trajectoryName.contains("mid")) {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, slow));
            } 
            else {
                trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, fast));
            }
        }

        builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP, translationKI, translationKD),
            new PIDConstants(rotationKP, rotationKI, rotationKD),
            swerve::setModuleStates,
            CommandEventMap,
            swerve
        );
    }

    public static CommandBase generateAuto(PathPlannerTrajectory trajectory) {
        return builder.fullAuto(trajectory);
    }

    public static CommandBase get(String name) {
        return builder.fullAuto(trajectories.get(name));
    }
}