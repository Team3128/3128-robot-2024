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
    private AmperStates state;
    private Elevator elevator;
    private Roller roller;

    public AmperExposed() {
        state = AmperStates.EXTENDED;
        transitionManager = new TransitionManager<>(AmperStates.class);
        elevator = Elevator.getInstance();
        roller = Roller.getInstance();
        registerTransitions();
    }

    public void pidTo(AmperStates state) {
        elevator.pidTo(state.getElevatorSetpoint());
        roller.pidTo(state.getRollerSetpoint());
    }

    public void registerTransitions() {
        transitionManager.addTransition(IDLE, PRIMED, runOnce(()-> pidTo(PRIMED)));
        transitionManager.addTransition(PRIMED, EXTENDED, runOnce(()-> pidTo(EXTENDED)));
        transitionManager.addTransition(EXTENDED, IDLE, runOnce(()-> pidTo(IDLE)));
    }

    public void setState(AmperStates nextState) {
        Transition<AmperStates> transition = transitionManager.getTransition(getState(), nextState);

        Log.info("State", "Check1");
        // if not the same state
        if(stateEquals(nextState)) Log.info("Amper", "State already set to " + nextState.name());

        // if invalid trnasition
        if(transition == null) Log.info("Amper", "State transition null");

        // if not transitioning
        Log.info("State", "Check2");
        Log.info("State", "Transitioning...");
        transition.execute();
        Log.info("Amper", "State transition successful. " + transition.toString());

        Log.info("Amper", "State transition failed");
    }

    public AmperStates getState() {
        return state;
    }

    public boolean stateEquals(AmperStates other) {
        return state.name().equals(other.name());
    }

}
