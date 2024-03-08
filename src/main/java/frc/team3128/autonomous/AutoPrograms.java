package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

import common.utility.narwhaldashboard.NarwhalDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.commands.CmdManager.autoShoot;

import java.util.HashMap;

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
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        // String selectedAutoName = "Test";
        final Command autoCommand;
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