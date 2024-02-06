package frc.team3128.commands;
import static frc.team3128.Constants.SwerveConstants.pigeonID;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;

//gyro log + setting speed for swerve based on setpoint
public class CmdSetSpeed extends Command {
    private DoubleSupplier setpoint;
    private Swerve swerve;
    private double prevPosition = 0;
    private double prevVelocity = 0;
    private double velocity = 0;
    private double acceleration = 0;
    private Pigeon2 gyro;
    private Supplier<Double> position;
    private Timer timer = new Timer();

    public CmdSetSpeed() {
        swerve = Swerve.getInstance();
        gyro = swerve.getGyro();
        initShuffleboard();
        addRequirements(swerve);
    }

    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.5)) {
            swerve.drive(new Translation2d(0,0), setpoint.getAsDouble(), false);
            velocity = (position.get() - prevPosition) / 0.5;
            prevPosition = position.get();
            acceleration = (velocity - prevVelocity) / 0.5;
            prevVelocity = velocity;
            timer.reset();
        }
        
    }


    public void initShuffleboard() {
        //widget for adjusting speed, ie. 1rad/sec
        setpoint = NAR_Shuffleboard.debug("gyroInfo", "setpoint", 0, 0, 0);
        //gyro log below
        //future purposes to see whether it reaches setpoint
        position = gyro.getYaw().asSupplier();
        //originally registering as an object instead of DoubleSupplier, so conver to double then double supplier lol
        NAR_Shuffleboard.addData("gyroInfo", "position", ()-> position.get(), 0, 0);
        NAR_Shuffleboard.addData("gyroInfo", "velocity", ()-> velocity,  1, 0);
        NAR_Shuffleboard.addData("gyroInfo", "acceleration", ()-> acceleration, 2, 0);


        }
    }

