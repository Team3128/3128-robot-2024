package frc.team3128.commands;

import static frc.team3128.Constants.FocalAimConstants.*;
import static frc.team3128.Constants.SwerveConstants.pigeonID;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import common.core.controllers.TrapController;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.core.commands.NAR_PIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team3128.subsystems.Swerve;

public class CmdFocalAim extends NAR_PIDCommand {
    private Swerve swerve;
    private DoubleSupplier setpoint;
    private DoubleSupplier measurement;
    
    private static TrapController controller = new TrapController(config, constraints); //put into constants folder 
    
    public CmdFocalAim(DoubleSupplier setpoint, DoubleSupplier measurement)
    { 
        super(controller, measurement, setpoint, (double output)-> Swerve.getInstance().drive(new Translation2d(0,0), Units.degreesToRadians(output), false));
        this.setpoint = setpoint;
        this.measurement = measurement;
        swerve = Swerve.getInstance();
    }
    @Override
    public void execute() {

    }

    
}

