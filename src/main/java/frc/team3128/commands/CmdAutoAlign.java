package frc.team3128.commands;
import static frc.team3128.Constants.LimelightConstants.*;
import static frc.team3128.Constants.SwerveConstants.maxSpeed;
import common.core.controllers.Controller;
import common.core.controllers.Controller.Type;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Swerve;



public class CmdAutoAlign extends Command{
    private double targetCount;
    private Controller controller;
    private double measurement;
    private double x_output;
    private double y_output;
    private double plateauCount;
    private double y_threshhold;
    private LimelightSubsystem m_limelight;
    private Swerve m_swerve;
    private Intake m_intake;
    private detectionStates targetState = detectionStates.SEARCHING;
    private enum detectionStates {
        SEARCHING, FEEDBACK, BLIND
    }

    public CmdAutoAlign() {
        //changed to trap controller
        controller = new Controller(config, Type.VELOCITY); //KD not used unless no friction involved, KI not used cause feedforward
        m_intake = Intake.getInstance();
        m_limelight = LimelightSubsystem.getInstance();
        m_swerve = Swerve.getInstance();
        addRequirements(m_swerve);
        

        //calls shuffleboard
        m_limelight.initShuffleboard();
    }
       @Override
       public void execute() {
        switch (targetState) {
            //count goes up every time an object is detect
            case SEARCHING:
            if (m_limelight.hasValidTarget()) //should be getValidTarget using area but common update first
            {
                targetCount ++;
            }
                else 
                {
                    targetCount = 0;
                    targetState = detectionStates.BLIND;
                }

            //tx_threshhold is the amount of interations needed for the image to not be blurry
            //essentially waits until the count is larger than threshhold, indicating that image is clear
            if (targetCount > TX_THRESHOLD) 
            { 
                //find exact threshhold later
                targetState = detectionStates.FEEDBACK;
                controller.reset();
            }
                break;
            
            //switches to searching state if no object is detected
            case FEEDBACK:

            if ( ! m_limelight.hasValidTarget()) { //also should be getValidtarget
                targetState = detectionStates.SEARCHING;
            }

            //if object is detected, calculate power needed to align 
            else 
            {
                m_intake.intake(Intake.Setpoint.EXTENDED); 
                m_intake.intakeRollers.intake();
                measurement = m_limelight.getObjectTX();
                x_output = controller.calculate(measurement);

                if (m_intake.intakeRollers.hasObjectPresent()) {
                    cancel();
                }

                if (x_output < maxSpeed) { //total speed equals max speed
                    y_output = maxSpeed - x_output;
                    m_swerve.drive(new Translation2d(x_output, y_output), 0, false);
                }

                else if (x_output >= maxSpeed) { //tx too large so just strafe to try to minimize error
                    m_swerve.drive(new Translation2d(maxSpeed, 0), 0, false);
                }
            
    
            }
                break;

            case BLIND:
            if (m_limelight.hasValidTarget()) { //also should be getValidTarget
                targetState = detectionStates.SEARCHING;
            }

            else if (m_limelight.getObjectTY() < y_threshhold) { //y decreases as you approach note. At some point, can't see note anymore
                plateauCount ++;
                
                if (plateauCount > 10) { //cancel command cause note prob taken
                    cancel();
                }
            }

            //resets error
            controller.reset();
                break;

        }

    }

       @Override
       public boolean isFinished() {
        return false;

       }

       @Override
       public void end(boolean interrupted) {
        m_swerve.stop();
       }

    

    
}
