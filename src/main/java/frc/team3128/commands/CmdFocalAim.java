package frc.team3128.commands;

import static frc.team3128.Constants.FocalAimConstants.*;
import java.util.function.DoubleSupplier;
import common.core.controllers.TrapController;
import common.core.commands.NAR_PIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team3128.subsystems.Swerve;

public class CmdFocalAim extends NAR_PIDCommand {
    private static TrapController controller;
    private Swerve swerve;
    private static DoubleSupplier measurement;
    private DoubleSupplier setpoint;
    private double rotation;
    
    public CmdFocalAim(DoubleSupplier setpoint)
    { 
        super(controller, measurement, setpoint, (double output)-> Swerve.getInstance().drive(new Translation2d(0,0), Units.degreesToRadians(output), false));
        this.setpoint = setpoint;
        swerve = Swerve.getInstance();
        controller = new TrapController(config, constraints); //put into constants folder 
      
    }
}

