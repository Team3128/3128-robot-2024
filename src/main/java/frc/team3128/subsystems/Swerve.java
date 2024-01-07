package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.utility.shuffleboard.NAR_Shuffleboard;

import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private WPI_Pigeon2 gyro;

    public double throttle = 1;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        super(swerveKinematics, Mod0, Mod1, Mod2, Mod3);
        gyro = new WPI_Pigeon2(pigeonID);
        initSwerveOdometry(SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD);
        for (SwerveModule module : modules) {
            NAR_Shuffleboard.addData("Swerve", "module " + module.moduleNumber, ()-> module.getCanCoder().getDegrees(), 0, module.moduleNumber);
        }
    }

    @Override
    public double getYaw() {
        return gyro.getYaw();
    }

    @Override
    public double getPitch() {
        return gyro.getPitch();
    }

    @Override
    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }

}