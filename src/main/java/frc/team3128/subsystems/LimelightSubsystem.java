package frc.team3128.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import common.hardware.limelight.Limelight;
import common.hardware.limelight.LimelightData;
import common.hardware.limelight.LimelightKey;
import common.hardware.limelight.Pipeline;
import common.utility.shuffleboard.NAR_Shuffleboard;
import static frc.team3128.Constants.LimelightConstants.*;

import java.util.function.BooleanSupplier;


//Class for the Limelight Subsystem 


public class LimelightSubsystem extends SubsystemBase{
    
    public static LimelightSubsystem instance;
    private Limelight m_limelight;

    public LimelightSubsystem() {
        m_limelight = new Limelight("limelight-teja", CAMERA_ANGLE, CAMERA_HEIGHT, FRONT_DISTANCE);
        //setting pipeline for detetecting both cube and cone

        /* update common with new pipeline after creating it
        Pipeline note = Pipeline.NOTE; 
        m_limelight.setPipeline(Pipeline.NOTE);
        */
    }

    public static synchronized LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    public void initShuffleboard() {
        // data that will appear under General Tab
        NAR_Shuffleboard.addData("General", "Range", this::calculateObjectDistance, 1, 3);
        // NAR_Shuffleboard.addData("General", "hasValidTarget", this::getValidTarget, 2, 2);
        NAR_Shuffleboard.addData("General", "ty", this::getObjectTY, 4, 1);
        NAR_Shuffleboard.addData("General", "tx", this::getObjectTX, 3, 1);
        NAR_Shuffleboard.addSendable("General", "LimelightInfo", this, 0,0);
        // data that will appear under Limelight Tab
        NAR_Shuffleboard.addData("Limelight", "ty", this::getObjectTY, 4, 1);
        NAR_Shuffleboard.addData("Limelight", "tx", this::getObjectTX, 3, 1);
        NAR_Shuffleboard.addSendable("Limelight", "LimelightInfo", this, 0,0);
    }
    
    //method to uniformly calculate distance to a ground target using a limelight
    public double calculateObjectDistance() {
        return m_limelight.calculateDistToGroundTarget(OBJ_TARGET_HEIGHT / 2);
    }
    
    //method to get limelight horizontal offset (tx) to target
    public double getObjectTX() {
        return m_limelight.getValue(LimelightKey.HORIZONTAL_OFFSET);
    }

    //getting shuffleboard data for cube (prob not necessary since only one color)
    // public boolean getisCube() {
    //     double[] data = m_limelight.getCustomData();
    //     return data[0] == 1.00;
    // }

     //method for getting limelight vertical offset (ty) to target
    public double getObjectTY() {
        return m_limelight.getValue(LimelightKey.VERTICAL_OFFSET);
    }

    //temporary method to avoid errors, replace with area one
    public boolean hasValidTarget() {
            return m_limelight.getValue(LimelightKey.VALID_TARGET) > 0.99;
    }

    //method for getting if the limelight has a valid target
    // public boolean getValidTarget() {

    //     if(m_limelight.getArea() > MIN_AREA){ //update common with the area method
    //         return m_limelight.hasValidTarget();
    //     }
    //     else{
    //         return false;
    //     }
    // }
    
    /**
     * Returns bottom-facing limelight object
     * @return ballLimelight object
     */
    public Limelight getObjectLimelight() {
        return m_limelight;
    }

 
    //use for updating common
    /* public double getArea() {
        return m_limelight.getValue(LimelightKey.AREA);
    }
    */ 
    }

