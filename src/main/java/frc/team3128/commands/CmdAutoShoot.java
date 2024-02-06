package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Shooter;

public class CmdAutoShoot extends Command {

    private Shooter shooter = Shooter.getInstance();
    private Climber climber = Climber.getInstance();
    
    public CmdAutoShoot() {

    }

    @Override
    public void initialize() {
        
    }
}
