package frc.team3128.commands;
import static frc.team3128.Constants.LimelightConstants.*;
import common.core.controllers.TrapController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.LimelightSubsystem;
import frc.team3128.subsystems.Swerve;



public class CmdAutoAlign extends Command{
    private double targetCount;
    private TrapController controller;
    private double measurement;
    private double output;
    private LimelightSubsystem m_limelight;
    private Swerve m_swerve;
    private Intake m_intake;
    private detectionStates targetState = detectionStates.SEARCHING;
    private enum detectionStates {
        SEARCHING, FEEDBACK, BLIND
    }

    public CmdAutoAlign() {
        //changed to trap controller
        controller = new TrapController(config, constraints); //KD not used unless no friction involved, KI not used cause feedforward
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
                m_intake.intake(Intake.Setpoint.EXTENDED); //done in parallel command?
                m_intake.intakeRollers.intake();
                measurement = m_limelight.calculateObjectDistance();
                output = controller.calculate(measurement);  //apparently distance prob not accurate..
                m_swerve.drive(new Translation2d(4, output), 0, false);
            }
                break;

            //basically rotate until an object is detected, then switch to searching
            case BLIND:
            m_swerve.drive(new Translation2d(0, 0), 3, false); 

            if (m_limelight.hasValidTarget()) { //also should be getValidTarget
                targetState = detectionStates.SEARCHING;
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
