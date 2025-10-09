package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;  
import frc.robot.constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class CANdleSubsystem extends SubsystemBase {
    public static CANdle staticCandle = new CANdle(constants.CanBus.CanBusIDs.CANdleID.id, constants.CanBus.RioCANBusName);
    private final CANdle m_candle ;
    private static final int LedCount = 300;
    private static final int BottomLightsSubtraction = 201;
    private final Timer m_LightstimeoutUntilResetToDefault = new Timer();
    //private XboxController joystick;
    int LightsTimeoutSeconds = 2;

    private static Animation m_toAnimate = null;
    
        public enum AnimationTypes {
            ColorFlow,
            Fire,
            Larson,
            Rainbow,
            RgbFade,
            SingleFade,
            Strobe,
            Twinkle,
            TwinkleOff,
            SetAll
        }
        private AnimationTypes m_currentAnimation;
    
        public CANdleSubsystem() {
            m_candle = CANdleSubsystem.staticCandle;
            //this.joystick = joystick2;
            changeAnimation(AnimationTypes.SetAll);
            CANdleConfiguration configAll = new CANdleConfiguration();
            configAll.statusLedOffWhenActive = true;
            configAll.disableWhenLOS = false;
            configAll.stripType = LEDStripType.GRB;
            configAll.brightnessScalar = 1.0;
            configAll.vBatOutputMode = VBatOutputMode.Modulated;
            m_candle.configAllSettings(configAll, 100);

            //first thing the robot does is set lights to default
            clearAnimations();
        }
    
        public void incrementAnimation() {
            switch(m_currentAnimation) {
                case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
                case Fire: changeAnimation(AnimationTypes.Larson); break;
                case Larson: changeAnimation(AnimationTypes.Rainbow); break;
                case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
                case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
                case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
                case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
                case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
                case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
                case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
            }
        }
        public void decrementAnimation() {
            switch(m_currentAnimation) {
                case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
                case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
                case Larson: changeAnimation(AnimationTypes.Fire); break;
                case Rainbow: changeAnimation(AnimationTypes.Larson); break;
                case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
                case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
                case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
                case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
                case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
                case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
            }
        }
        public void setColors() {
            changeAnimation(AnimationTypes.SetAll);
        }
    
        /* Wrappers so we can access the CANdle from the subsystem */
        public double getVbat() { return m_candle.getBusVoltage(); }
        public double get5V() { return m_candle.get5VRailVoltage(); }
        public double getCurrent() { return m_candle.getCurrent(); }
        public double getTemperature() { return m_candle.getTemperature(); }
        public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
        public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
        public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
        public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }
    
        public void changeAnimation(AnimationTypes toChange) {
            m_currentAnimation = toChange;
            
            switch(toChange)
            {
                case ColorFlow:
                    m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                    break;
                case Fire:
                    m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                    break;
                case Larson:
                    m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                    break;
                case Rainbow:
                    m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                    break;
                case RgbFade:
                    m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                    break;
                case SingleFade:
                    m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                    break;
                case Strobe:
                    m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                    break;
                case Twinkle:
                    m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 1.00, LedCount, TwinklePercent.Percent64);
                    break;
                case TwinkleOff:
                    m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                    break;
                case SetAll:
                    m_toAnimate = null;
                    break;
            }
            System.out.println("Changed to " + m_currentAnimation.toString());
        }
    
        @Override
        public void periodic() {
            // This method will be called once per scheduler run
            if(m_toAnimate == null) {
          //      m_candle.setLEDs((int)(joystick.getLeftTriggerAxis() * 255), 
          //                        (int)(joystick.getRightTriggerAxis() * 255), 
           //                       (int)(joystick.getLeftX() * 255));
            } else {
                m_candle.animate(m_toAnimate);
            }
           // m_candle.modulateVBatOutput(joystick.getRightY());
           if(m_LightstimeoutUntilResetToDefault.get() > LightsTimeoutSeconds){
                clearLayer0();
                defaultLights();
                m_LightstimeoutUntilResetToDefault.stop();
                m_LightstimeoutUntilResetToDefault.reset();
    
           }
        }
    
        @Override
        public void simulationPeriodic() {
            // This method will be called once per scheduler run during simulation
        }
    
        public void clearLayer0(){
            m_toAnimate = null;
            m_candle.clearAnimation(0);
        }
        
        public static void clearAnimations() {
            m_toAnimate = null;
            for(int i = 0; i < 10; ++i) {
                    staticCandle.clearAnimation(i);
            }
            defaultLights();
        }
        public static void defaultLights(){
        // var m_toAnimate0 = new SingleFadeAnimation(0, 0, 255, 0, 0.3, LedCount);

        // m_candle.animate(m_toAnimate0, 0);
        staticCandle.setLEDs(0, 0, 255, 0, 0, LedCount);
        staticCandle.configBrightnessScalar(1.0);
    }
    public static void fireLights(){
        var m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        staticCandle.animate(m_toAnimate,0);
    }
    public static void timeToClimbLights(){
        var m_toAnimate = new TwinkleAnimation(255, 0, 0, 0, 1.00, LedCount, TwinklePercent.Percent88);
        staticCandle.animate(m_toAnimate, 0);
    }
    public static void limelightBypassLights(){
        var m_toAnimate = new StrobeAnimation(0, 250, 0, 0, .5, LedCount);
        staticCandle.animate(m_toAnimate,0);
    }

    public static void climbLights(){
        //Alliance Specific Climbing Lights
        if(DriverStation.getAlliance().get() == Alliance.Red){
            var  m_toAnimate = new StrobeAnimation(255, 0, 0, 0, .3, LedCount - BottomLightsSubtraction);
            staticCandle.animate(m_toAnimate, 0);
        }else{
            var  m_toAnimate = new StrobeAnimation(0, 0, 255, 0, .3, LedCount - BottomLightsSubtraction);
            staticCandle.animate(m_toAnimate, 0);}
         

        staticCandle.setLEDs(255, 55, 0, 0, LedCount - BottomLightsSubtraction, LedCount);
        staticCandle.configBrightnessScalar(1);
    }
    public static void redLights(){
        staticCandle.setLEDs(255, 0, 0, 0, 0, LedCount);
        staticCandle.configBrightnessScalar(1.0);
    }
    public void lightsOne(){
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
        m_LightstimeoutUntilResetToDefault.restart();
    }
    public void lightsTwo(){
        m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    }
    public void lightsFinished(){
        m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
        m_LightstimeoutUntilResetToDefault.restart();

    }
    public void lightsTrap(){
        m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
    }
    public void resetToDefault() {
        
    }

    public static void noAprilTagOnBoot_strobeRed(){
            var m_toAnimate = new StrobeAnimation(255, 0, 0, 0, .45, LedCount);
            staticCandle.animate(m_toAnimate,0);
            staticCandle.animate(m_toAnimate,1);
            staticCandle.animate(m_toAnimate,2);
            staticCandle.animate(m_toAnimate,3);
            staticCandle.animate(m_toAnimate,4);
            staticCandle.animate(m_toAnimate,5);
  
    }
     public static void noAprilTagOnBoot_strobeWhite(){
            var m_toAnimate = new StrobeAnimation(255, 255, 255, 255, .55, LedCount);
            staticCandle.animate(m_toAnimate,0);
            staticCandle.animate(m_toAnimate,1);
            staticCandle.animate(m_toAnimate,2);
            staticCandle.animate(m_toAnimate,3);
            staticCandle.animate(m_toAnimate,4);
            staticCandle.animate(m_toAnimate,5);
  
    }
         public static void noAprilTagOnBoot_strobePurple(){
            var m_toAnimate = new StrobeAnimation(255, 0, 255, 0, .10, LedCount);
            staticCandle.animate(m_toAnimate,0);
            staticCandle.animate(m_toAnimate,1);
            staticCandle.animate(m_toAnimate,2);
            staticCandle.animate(m_toAnimate,3);
            staticCandle.animate(m_toAnimate,4);
            staticCandle.animate(m_toAnimate,5);
  
    }
    public static void noAprilTagOnBoot_MightBeOkYellow(){
            var m_toAnimate = new ColorFlowAnimation(255, 255, 0, 0, .55, LedCount, Direction.Forward);
            staticCandle.animate(m_toAnimate,0);
            staticCandle.animate(m_toAnimate,1);
            staticCandle.animate(m_toAnimate,2);
            staticCandle.animate(m_toAnimate,3);
            staticCandle.animate(m_toAnimate,4);
            staticCandle.animate(m_toAnimate,5);
  
    }
}
