package frc.team3128.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import common.utility.narwhaldashboard.NarwhalDashboard;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public AutoPrograms() {

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {

                                            };
        NarwhalDashboard.getInstance().addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        return null;
    }
}