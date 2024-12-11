package frc.team3128.subsystems.Amper;

import common.core.subsystems.Transition;
import common.core.subsystems.TransitionManager;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.subsystems.Amper.AmperStates.*;

public class AmperExposed extends SubsystemBase{
    
    private static AmperExposed instance;

    public static synchronized AmperExposed getInstance() {
        if (instance == null) {
            instance = new AmperExposed();
        }
        return instance;
    }

    private TransitionManager<AmperStates> transitionManager;
    private Transition<AmperStates> lastScheduledTransition;
    private AmperStates state;
    private Elevator elevator;
    private Roller roller;

    public AmperExposed() {
        state = AmperStates.IDLE;
        transitionManager = new TransitionManager<>(AmperStates.class);
        elevator = Elevator.getInstance();
        roller = Roller.getInstance();
        registerTransitions();
    }

    public void pidTo(AmperStates state) {
        elevator.pidTo(state.getElevatorSetpoint()).schedule();
        roller.pidTo(state.getRollerSetpoint()).schedule();
    }

    public void registerTransitions() {
        transitionManager.addTransition(IDLE, PRIMED, runOnce(()-> {pidTo(PRIMED); state = PRIMED;}));
        transitionManager.addTransition(PRIMED, EXTENDED, runOnce(()-> {pidTo(EXTENDED); state = EXTENDED;}));
        transitionManager.addTransition(EXTENDED, IDLE, runOnce(()-> {pidTo(IDLE); state = IDLE;}));
    }

    public void setState(AmperStates nextState) {
        Log.info("COMMANDED", state.name() + " -> " + nextState.name());
        // if not the same state
        if(stateEquals(nextState)) {
            Log.info("COMMANDED", "State already set to " + nextState.name());
            return;
        }

        Transition<AmperStates> transition = transitionManager.getTransition(getState(), nextState);
        // if invalid trnasition
        if(transition == null) {
            Log.info("TRANSITION", "State transition null");
            return;
        }
        Log.info("TRANSITION", transition.toString());

        if(isTransitioning()) {
            Log.info("TRANSITION", "Already transitioning, procceeding to override");
            lastScheduledTransition.cancel();
        }

        state = nextState;
        lastScheduledTransition = transition;
        lastScheduledTransition.execute();

        if(isTransitioning()) Log.info("State", "Transitioning...");
    }

    public boolean isTransitioning() {
        return lastScheduledTransition != null && lastScheduledTransition.isRunning();
    }

    public AmperStates getState() {
        return state;
    }

    public boolean stateEquals(AmperStates other) {
        return state.name().equals(other.name());
    }
}
