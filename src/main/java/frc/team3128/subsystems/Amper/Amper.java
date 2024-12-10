package frc.team3128.subsystems.Amper;

import common.core.subsystems.StateSubsystemBase;
import common.core.subsystems.TransitionManager;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.team3128.subsystems.Amper.AmperStates.*;

public class Amper extends StateSubsystemBase<AmperStates> {
    
    private Elevator elevator;
    private Roller roller;
    
    private static Amper instance;

    public static synchronized Amper getInstance() {
        if (instance == null) {
            instance = new Amper();
        }
        return instance;
    }

    private Amper() {
        super(IDLE, AmperStates.class);
        elevator = Elevator.getInstance();
        roller = Roller.getInstance();
        addSubsystems(elevator, roller);
    }

    public void registerTransitions() {
        getTransitionManager().applyConvergingFunction((AmperStates S) -> pidTo(S), EXTENDED, PRIMED, IDLE);
        getTransitionManager().addTransition(IDLE, EXTENDED, pidTo(EXTENDED));
        getTransitionManager().addTransition(EXTENDED, IDLE, pidTo(IDLE));
        // getTransitionManager().addTransition(IDLE, SYS_ID, elevator.characterization(3, 1));
    }

    public Command pidTo(AmperStates state) {
        if (elevator == null || roller == null) return Commands.none();
        return sequence(
            elevator.pidTo(state.getElevatorSetpoint()),
            roller.pidTo(state.getRollerSetpoint())
        );
    }
}
