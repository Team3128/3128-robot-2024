package frc.team3128.subsystems.Amper;

import common.core.subsystems.StateSubsystemBase;
import common.core.subsystems.TransitionManager;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; 
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.team3128.subsystems.Amper.AmperStates.*;

public class Amper extends StateSubsystemBase<AmperStates>{
    
    private Elevator elevator;
    private Roller roller;
    
    private Amper instance;

    public Amper getInstance() {
        if (instance == null) {
            instance = new Amper();
        }
        return instance;
    }

    public Amper() {
        super(UNDETERMINED, AmperStates.class);
        elevator = new Elevator();
        roller = new Roller();
        addSubsystems(elevator, roller);
    }

    public void registerTransitions() {
        getTransitionManager().applyConvergingFunction((AmperStates S) -> pidTo(S), IDLE, RETRACTED, ALIGNING, EXTENDED, AMPING);
        getTransitionManager().addTransition(IDLE, SYS_ID, elevator.characterization(3, 1));
    }

    public Command pidTo(AmperStates state) {
        return sequence(
            elevator.pidTo(state.getElevatorSetpoint()),
            roller.pidTo(state.getRollerSetpoint())
        );
    }

}
