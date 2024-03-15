// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.commands.CmdManager;
import frc.team3128.subsystems.Leds;
import frc.team3128.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {

    private boolean hasInitialized = false;

    public static Alliance alliance;

    public static Alliance getAlliance() {
        if (alliance == null) {
            Optional<Alliance> DSalliance = DriverStation.getAlliance();
            if (DSalliance.isPresent()) alliance = DSalliance.get();
        }
        return alliance;
    }

    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    public static AutoPrograms autoPrograms;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        autoPrograms = new AutoPrograms();
        m_robotContainer.initDashboard();
        LiveWindow.disableAllTelemetry();
        // runOnce(()-> Swerve.getInstance().zeroGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180));

        addReceiver(true, LoggingState.SESSION);
    }

    @Override
    public void robotPeriodic(){
        Camera.updateAll();
    }

    @Override
    public void autonomousInit() {
        Leds.getInstance().setDefaultColor();
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        if(DriverStation.getMatchType() != MatchType.None)
            addReceiver(true, LoggingState.FULLMATCH);
        Logger.start();

    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        if(DriverStation.getMatchType() != MatchType.None)
            addReceiver(true, LoggingState.FULLMATCH);
        Logger.start();
        
        CmdManager.neutral(false).schedule();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {
        
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Swerve.getInstance().setBrakeMode(true);
        CommandScheduler.getInstance().cancelAll();
        sequence(
            waitSeconds(3.0).ignoringDisable(true),
            runOnce(()->Swerve.getInstance().setBrakeMode(false)).ignoringDisable(true)
        ).schedule();

        if (hasInitialized) {
            Leds.getInstance().setLedColor(Colors.AMP);
        }
        hasInitialized = true;
    }

    @Override
    public void disabledExit() {
        Leds.getInstance().setDefaultColor();
        Swerve.getInstance().setBrakeMode(false);
    }
    
    @Override
    public void disabledPeriodic() {

    }
}
