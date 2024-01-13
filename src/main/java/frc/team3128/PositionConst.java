package main.java.frc.team3128;

public class PositionConst {
    public static enum Position{
        //Outake
        SPEAKER(0,0),
        AMP(0,0),
        TRAP(0,0),

        //Intake
        GROUND(0,0),
        SOURCE(0,0);

        public double pivAngle;
        public double wristAngle;

        private Position(double pivAngle, double wristAngle){
            this.pivAngle = pivAngle;
            this.wristAngle = wristAngle;
        }
    }
}
