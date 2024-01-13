package main.java.frc.team3128.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveModule;
import common.utility.shuffleboard.NAR_Shuffleboard;

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

}