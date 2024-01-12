package frc.team3128.autonomous;

import java.util.HashMap;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static final HashMap<String, PathPlannerPath> trajectories = new HashMap<String, PathPlannerPath>();

    public static void initTrajectories() {
        final String[] trajectoryNames = {};

        for (final String trajectoryName : trajectoryNames) {

            if (trajectoryName.contains("mid")) {
                trajectories.put(trajectoryName, PathPlannerPath.fromPathFile(trajectoryName));
            } 
            else {
                trajectories.put(trajectoryName, PathPlannerPath.fromPathFile(trajectoryName));
            }
        }

        // AutoBuilder.configureHolonomic(
        //     swerve::getPose,
        //     swerve::resetOdometry,
        //     swerve::getRobotVelocity,
        //     swerve::drive,
        //     swerveKinematics,
        //     CommandEventMap,
        //     swerve
        // );
    }

    // public static Command generateAuto(PathPlannerTrajectory trajectory) {
    //     return AutoBuilder.fullAuto(trajectory);
    // }

    // public static Command get(String name) {
    //     return builder.fullAuto(trajectories.get(name));
    // }
}