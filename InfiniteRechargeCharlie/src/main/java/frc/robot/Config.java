package frc.robot;

public class Config {
    //Power
    public static final double beltPower = 0.6;
    public static final double feederIntakePower = 0.7;
    public static final double feederExhaustPower = 0.7;
    //Limelight
    public static final double Kp = 0.007;
    public static final double mountAngle = 25.0;

    public static final double cameraHeight = 1.168; //Camera height in meters
    public static final double rcHeight = 2.4892; //hight of the goal 
    public static final double hpHeight = 2.4892; //Hatch panel height in meters
    public static final double scHeight = 2.4892; //Cargo ship height in meters

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static boolean kSensorPhase = true;
    public static boolean kMotorInvert = false;
   
    //static final Gains kGains = new Gains(0.15, 0.0, 1, 0, 0, 1);

}