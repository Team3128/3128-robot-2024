package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.Constants.FieldConstants;

public enum AutoStates {
    IDLE(),
    RAMSHOT(),
    NOTE1a(),
    NOTE1b();

    private Translation2d translation;

    private AutoStates(Translation2d translation) {
        this.translation = translation;
    }

    private AutoStates() {
        this.translation = new Translation2d();
    }

    public Translation2d getTranslation() {
        return FieldConstants.allianceFlip(translation);
    }
}
