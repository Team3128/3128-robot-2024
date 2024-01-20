package frc.team3128.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.Robot;

import static frc.team3128.Constants.FocalAimConstants.FIELD_Y_LENGTH;
import static frc.team3128.Constants.FocalAimConstants.speakerLength;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.Constants.FocalAimConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public double throttle = 1;

    public Supplier<Double> yaw, pitch, roll;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        super(swerveKinematics, Mod0, Mod1, Mod2, Mod3);
        gyro = new Pigeon2(pigeonID);
        yaw = gyro.getYaw().asSupplier();
        pitch = gyro.getPitch().asSupplier();
        roll = gyro.getRoll().asSupplier();
        initSwerveOdometry(SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD);
        for (SwerveModule module : modules) {
            NAR_Shuffleboard.addData("Swerve", "module " + module.moduleNumber, ()-> module.getCanCoder().getDegrees(), 0, module.moduleNumber);
        }
    }

    @Override
    public double getYaw() {
        return yaw.get();
    }
    


    @Override
    public double getPitch() {
        return pitch.get();
    }

    @Override
    public double getRoll() {
        return roll.get();
    }

    @Override
    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }
    
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public double getDist(Pose2d aimPoint){
        Pose2d robotPosition = Swerve.getInstance().getPose();
        double coordRobotX = robotPosition.getTranslation().getX();
        double coordRobotY = robotPosition.getTranslation().getY();
        double coordFocalX = aimPoint.getTranslation().getX();
        double coordFocalY = aimPoint.getTranslation().getY();
        double distance = Math.sqrt(Math.pow((coordFocalX-coordRobotX),2)+Math.pow((coordFocalY-coordRobotY),2));
        return distance;

    }
    public Pose2d getAimPoint() {
        Pose2d robotPosition = Swerve.getInstance().getPose();
        double coordRobotYInteger = robotPosition.getTranslation().getY() - (8.21/2);
        double[] focalPoint;
        
        if (Robot.getAlliance() == Alliance.Blue ){
            double[] midpointCoordSpeaker = {midPointSpeakerBlue.getTranslation().getX(), midPointSpeakerBlue.getTranslation().getY()};
            double ratioRobot = coordRobotYInteger/FIELD_Y_LENGTH;
            focalPoint = new double[2];
            if (coordRobotYInteger > 0) {
                focalPoint[1] = -1*((speakerLength/2) * ratioRobot)+ 6.057+ (speakerLength/2);
                focalPoint[0] = midpointCoordSpeaker[0];
            } 

            else if (coordRobotYInteger < 0) { 
                focalPoint[1] = ((speakerLength/2) * ratioRobot)+ 6.057+ (speakerLength/2);
                focalPoint[0] = midpointCoordSpeaker[0];
            }
            return new Pose2d(focalPoint[0], focalPoint[1], new Rotation2d(0));
        }

        else{
            double[] midpointCoordSpeaker = {midPointSpeakerRed.getTranslation().getX(), midPointSpeakerRed.getTranslation().getY()};
            double ratioRobot = coordRobotYInteger/FIELD_Y_LENGTH;
            focalPoint = new double[2];
            if (coordRobotYInteger > 0) {
                focalPoint[1] = -1*((speakerLength/2) * ratioRobot) + 6.057+ (speakerLength/2);
                focalPoint[0] = midpointCoordSpeaker[0];
            }

            else if (coordRobotYInteger < 0) { 
                focalPoint[1] = ((speakerLength/2) * ratioRobot) + 6.057+ (speakerLength/2);
                focalPoint[0] = midpointCoordSpeaker[0];
            }
            return new Pose2d(focalPoint[0], focalPoint[1], new Rotation2d(0));
    }
}

}