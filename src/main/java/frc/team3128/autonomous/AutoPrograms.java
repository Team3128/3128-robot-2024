package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Intake;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.commands.CmdManager.autoShoot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();

    public AutoPrograms() {

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {
            "top_2note",
            "top_4note",
            "topFar_4note",
            "topFarCopy_4note",
            "middle_2note",
            "middle_4note",
            "middleClose_4note",
            "middle_6note",
            "bottom_2note",
            "bottom_4note",
            "bottom_7note",
            "bottomClose_4note",
            "special_3note",
            "leave"
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
    }

    public Command preload() {
        return sequence(
            deadline(
                race(
                    Intake.getInstance().intakeAuto(),
                    Trajectories.align(false)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        Command autoCommand;
        if (selectedAutoName == null) {
            autoCommand = sequence(
                Trajectories.resetAuto(),
                waitSeconds(5),
                autoShoot()
            );
        }
        else {
            autoCommand = autoMap.get(selectedAutoName);
        }

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }
}