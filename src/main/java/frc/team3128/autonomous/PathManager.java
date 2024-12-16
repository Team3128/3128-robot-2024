package frc.team3128.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import common.core.subsystems.Transition;
import common.core.subsystems.TransitionManager;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.Robot;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.autonomous.AutoStates.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class PathManager {

    private static PathManager instance;

    public static synchronized PathManager getInstance() {
        if (instance == null) {
            instance = new PathManager();
        }
        return instance;
    }
    
    private static Swerve swerve;
    private static TransitionManager<AutoStates> transitionManager = new TransitionManager<>(AutoStates.class);
    private static final List<String> paths = new ArrayList<>();
    private Transition<AutoStates> lastScheduledTransition;
    private AutoStates state = IDLE;
    private BooleanSupplier hasNote = NAR_Shuffleboard.debug("Autos", "Has Note", true, 0, 0);

    static {
        AutoBuilder.configureHolonomic(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::getRobotVelocity,
            Swerve.getInstance()::driveOverridable,
            new HolonomicPathFollowerConfig(
                new PIDConstants(3, 0, 0),
                new PIDConstants(3, 0, 0),
                2,
                0.66,
                new ReplanningConfig(false, true)
            ),
            ()-> Robot.getAlliance() == Alliance.Red,
            Swerve.getInstance()
        );


        paths.add("RAMSHOT_TO_NOTE1a");
        paths.add("NOTE1a_TO_RAMSHOT");
        paths.add("RAMSHOT_TO_NOTE1b");
        paths.add("NOTE1b_TO_RAMSHOT");
        paths.add("NOTE1a_TO_NOTE1b");

        NamedCommands.registerCommand("INTAKE", Commands.print("INTAKE AUTO RUNNING"));
        NamedCommands.registerCommand("RAMP", Commands.print("RAMP AUTO RUNNING"));

    }

    public PathManager() {
        state = IDLE;
        swerve = Swerve.getInstance();
        registerTransitions();
    }

    public void registerTransitions() {
        for(String path : paths) {
            String[] pathSplit = path.split("_TO_");
            transitionManager.addTransition(
                AutoStates.valueOf(pathSplit[0]), 
                AutoStates.valueOf(pathSplit[1]), 
                getPathCommand(path)
            );
        }
    }

    public Command getPathCommand(String path) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
    }

    public Command setState(AutoStates nextState) {
        Log.info("COMMANDED", state.name() + " -> " + nextState.name());
        // if not the same state
        if(stateEquals(nextState)) {
            Log.info("COMMANDED", "State already set to " + nextState.name());
            return none();
        }

        Transition<AutoStates> transition = transitionManager.getTransition(getState(), nextState);
        // if invalid trnasition
        if(transition == null) {
            Log.info("TRANSITION", "State transition null");
            return none();
        }
        Log.info("TRANSITION", transition.toString());

        if(isTransitioning()) {
            Log.info("TRANSITION", "Already transitioning, procceeding to override");
            lastScheduledTransition.cancel();
        }

        state = nextState;
        lastScheduledTransition = transition;
        if(isTransitioning()) Log.info("State", "Transitioning...");
        return lastScheduledTransition.getCommand();
    }

    public boolean isTransitioning() {
        return lastScheduledTransition != null && lastScheduledTransition.isRunning();
    }

    public AutoStates getState() {
        return state;
    }

    public boolean stateEquals(AutoStates other) {
        return state.name().equals(other.name());
    }

    public Command topOnePiece() {
        return sequence(
            setState(RAMSHOT),
            print("Shoot Preload"),
            setState(NOTE1a),
            setState(RAMSHOT).andThen(print("Shoot 1a")).onlyIf(hasNote) /*has note*/,
            setState(NOTE1b),
            setState(RAMSHOT).andThen(print("Shoot 1b")).onlyIf(hasNote) /*has note*/
        );
    }

}
