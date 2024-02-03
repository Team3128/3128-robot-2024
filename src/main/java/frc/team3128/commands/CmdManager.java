package frc.team3128.commands;

import static frc.team3128.Constants.FocalAimConstants.*;

import common.core.commands.NAR_PIDCommand;
import common.core.controllers.TrapController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;


public class CmdManager {
    public static Command CmdTurnInPlace() {
        TrapController trap = new TrapController(config, constraints);
        trap.enableContinuousInput(0, 360);
        trap.setTolerance(0.5);
        return new NAR_PIDCommand(trap, 
        ()-> Swerve.getInstance().getYaw(), //measurement
        ()-> Swerve.getInstance().getTurnAngle(focalPoint), //setpoint
        (double output) -> Swerve.getInstance().drive(new Translation2d(), Units.degreesToRadians(output), false),
        Swerve.getInstance());
    }
}