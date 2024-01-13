package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ManipulatorConstants.STALL_POWER;

import frc.team3128.subsystems.Manipulator;
import frc.team3128.subsystems.Pivot;
import frc.team3128.subsystems.PivotTrap;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.ShooterTrap;
import frc.team3128.subsystems.Telescope;


public class CmdManager {
    public static Manipulator manipulator = Manipulator.getInstance();
    public static Pivot pivot = Pivot.getInstance();
    public static PivotTrap pivotTrap = PivotTrap.getInstance();
    public static Shooter shooter = Shooter.getInstance();
    public static ShooterTrap shooterTrap = ShooterTrap.getInstance();
    public static Telescope telescope = Telescope.getInstance();
    
    
    public static CommandBase Outtake() {
        return sequence (
            runOnce(() -> manipulator.outtake()),
            waitUntil(() -> !manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopManip())
        );
    }
    
    public static CommandBase Intake() {
        return sequence (
            runOnce(() -> manipulator.intake()),
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> manipulator.stopManip()),
            runOnce(() -> manipulator.setPower(STALL_POWER))
        );
    }
    
    public static CommandBase Shoot() {
        return sequence (
            waitUntil(() -> manipulator.hasObjectPresent()),
            runOnce(() -> shooterTrap.shoot(200))
        );
    }
    
}