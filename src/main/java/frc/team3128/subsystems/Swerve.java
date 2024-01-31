package frc.team3128.subsystems;

import static frc.team3128.Constants.SwerveConstants.Mod0;
import static frc.team3128.Constants.SwerveConstants.Mod1;
import static frc.team3128.Constants.SwerveConstants.Mod2;
import static frc.team3128.Constants.SwerveConstants.Mod3;
import static frc.team3128.Constants.SwerveConstants.pigeonID;
import static frc.team3128.Constants.SwerveConstants.swerveKinematics;
import static frc.team3128.Constants.VisionConstants.SVR_STATE_STD;
import static frc.team3128.Constants.VisionConstants.SVR_VISION_MEASUREMENT_STD;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.swerve.SwerveBase;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

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

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        gyro = new Pigeon2(pigeonID);
        yaw = gyro.getYaw().asSupplier();
        pitch = gyro.getPitch().asSupplier();
        roll = gyro.getRoll().asSupplier();
        initShuffleboard();
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

    public double getDist(Translation2d aimPoint) {
         Pose2d robotPosition = Swerve.getInstance().getPose();
        double coordRobotX = robotPosition.getTranslation().getX();
        double coordRobotY = robotPosition.getTranslation().getY();
        double coordFocalX = aimPoint.getX();
        double coordFocalY = aimPoint.getY();
        double distance = Math.sqrt((coordRobotY-coordFocalY) * (coordRobotY-coordFocalY) + (coordRobotX-coordFocalX) * (coordRobotX-coordFocalX));
        return distance;
    }

    public double getTurnAngle(Translation2d aimPoint) {
        Pose2d robotPosition = Swerve.getInstance().getPose();
        double coordRobotX = robotPosition.getTranslation().getX();
        double coordRobotY = robotPosition.getTranslation().getY();
        double coordFocalX = aimPoint.getX();
        double coordFocalY = aimPoint.getY();
        double angle = Math.toDegrees(Math.atan2(coordRobotY-coordFocalY, coordRobotX-coordFocalX));
        return angle;
    }


        }
    

