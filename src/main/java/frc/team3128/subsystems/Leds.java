package frc.team3128.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3128.Constants.LedConstants.*;

import frc.team3128.Constants.LedConstants;
import frc.team3128.Constants.LedConstants.Colors;


public class Leds extends SubsystemBase {
    private final CANdle m_candle = new CANdle(CANDLE_ID);

    private static Leds instance;

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds();
        }
        return instance;
    }

    public Leds() {
        configCandle();
    }

    private void configCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 1;
        m_candle.configAllSettings(config);
    }

    public void setDefaultColor() {
        setLedColor(Colors.FLAME);
        //setLedColor(Robot.getAlliance() == Alliance.Red ? Colors.RED : Colors.BLUE);
    }

    //Set Elevator Leds
    public void setLedColor(Colors color) {

        switch (color) {
            case AMP:
                resetAnimationSlot(2);
                m_candle.animate(new RainbowAnimation(BRIGHTNESS, r_SPEED, PIVOT_FRONT, false, STARTING_ID), 0);
                m_candle.animate(new RainbowAnimation(BRIGHTNESS, r_SPEED, PIVOT_BACK, true, STARTING_ID + PIVOT_FRONT), 1);
                break;
            case FLAME:
                resetAnimationSlot(2);
                m_candle.animate(new FireAnimation(BRIGHTNESS, r_SPEED, NUM_LED, SPARKING, COOLING, false, 5), 0);
                m_candle.animate(new FireAnimation(BRIGHTNESS,r_SPEED, NUM_LED, SPARKING, COOLING, true, OFFSET), 1);
                break;
            case CHARGE:
                resetAnimationSlot(2);
                m_candle.animate(new ColorFlowAnimation(color.r, color.g, color.b, WHITE_VALUE, 1, 0, ColorFlowAnimation.Direction.Forward, 5), 0);
                m_candle.animate(new ColorFlowAnimation(color.r, color.g, color.b, WHITE_VALUE, 1, 0, ColorFlowAnimation.Direction.Backward, OFFSET), 1);
                break;
            case DISCHARGE:
                // resetAnimationSlot(2);
                m_candle.animate(new ColorFlowAnimation(color.r, color.g, color.b, WHITE_VALUE, 1, 0, ColorFlowAnimation.Direction.Backward, 5), 0);
                m_candle.animate(new ColorFlowAnimation(color.r, color.g, color.b, WHITE_VALUE, 1, 0, ColorFlowAnimation.Direction.Forward, OFFSET), 1);
                break;
            case PIECE:
            case ERROR:
                resetAnimationSlot(2);
                m_candle.animate(new SingleFadeAnimation(color.r, color.g, color.b,WHITE_VALUE, HOLDING_SPEED,PIVOT_COUNT), 0);
                break;
            default:
                resetAnimationSlot(2);
                m_candle.setLEDs(color.r,color.g,color.b, WHITE_VALUE, STARTING_ID, PIVOT_COUNT);
                break;
        }
    }

    public void resetAnimationSlot(int slots) {
        m_candle.setLEDs(0,0,0, WHITE_VALUE, STARTING_ID, PIVOT_COUNT);
        for (int i = 0; i < slots; i++) {
         m_candle.animate(null, i);
        }
     }

    public void resetAnimationSlot(int slots, int offset) {
       for (int i = 0; i < slots; i++) {
        m_candle.animate(null, i+offset);
       }
    }    

}