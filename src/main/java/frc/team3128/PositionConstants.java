package frc.team3128;

public class PositionConstants {
    public static enum Positions {
        //intake
        GROUND(0,0),
        SOURCE(0,0),

        //outake
        SPEAKER(0,0),
        AMP(0,0),
        TRAP(0,0);

        public double pivotANGLE;
        public double wristANGLE;

        private Positions(double pivotANGLE, double wristANGLE) {
            this.pivotANGLE = pivotANGLE;
            this.wristANGLE = wristANGLE;
        }
    }
}