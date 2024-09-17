
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.runner.RunWith;

import frc.team3128.subsystems.Intake;

public class VisionTest {


   @Test
   public void getRunningStateTest() {
       Intake intake = Intake.getInstance();
       Assertions.assertTrue(false);
       // if (PIVOT_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
       // if (LEFT_ROLLER_MOTOR.getState() == State.DISCONNECTED && RIGHT_ROLLER_MOTOR.getState() == State.DISCONNECTED) return State.DISCONNECTED;
       // if (LEFT_ROLLER_MOTOR.getState() == State.DISCONNECTED || RIGHT_ROLLER_MOTOR.getState() == State.DISCONNECTED) return State.PARTIALLY_RUNNING;
       // return State.RUNNING;
   }


  
}
