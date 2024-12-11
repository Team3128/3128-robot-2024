package frc.team3128.subsystems.Amper;

import common.core.subsystems.StateSubsystemBase;
import common.core.subsystems.TransitionManager;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
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
        state = AmperStates.IDLE;
        addSubsystems(elevator, roller);
    }

    public void registerTransitions() {
        // getTransitionManager().applyConvergingFunction((AmperStates S) -> pidTo(S), EXTENDED, PRIMED, IDLE);
        getTransitionManager().addTransition(IDLE, EXTENDED, runOnce(()-> pidTo(EXTENDED)));
        getTransitionManager().addTransition(EXTENDED, IDLE, runOnce(()-> pidTo(IDLE)));
        // getTransitionManager().addTransition(IDLE, SYS_ID, elevator.characterization(3, 1));
    }

    public void pidTo(AmperStates state) {
        elevator.pidTo(state.getElevatorSetpoint()).schedule();
        roller.pidTo(state.getRollerSetpoint()).schedule();
        this.state = state;
        Log.info("State", "State set to: " + state.name());
    }

    // public Command pidTo(AmperStates state) {
    //     if (elevator == null || roller == null) return Commands.none();
    //     Command ret = parallel(
    //         elevator.pidTo(state.getElevatorSetpoint()),
    //         roller.pidTo(state.getRollerSetpoint())
    //     ).andThen(waitUntil(()-> elevator.atSetpoint() && roller.atSetpoint()))
    //     .withName("pidTo");
    //     ret.addRequirements(this);
    //     return ret;
    // }
}
