package frc.robot;

//this is essentially an extension of the constants class but ctrl c + ctrl v is hard
public class Config {

    //Limelight Variables 
    public static final double Kp = 0.007;
    public static final double mountAngle = 25.0;

    public static final double cameraHeight = 2.3; //Camera height in meters
    public static final double rcHeight = 2.4892; //hight of the goal 
    public static final double hpHeight = 8.1875; //Hatch panel height in meters
    public static final double scHeight = 2.4892; //Cargo ship height in meters

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static boolean kSensorPhase = true;
    public static boolean kMotorInvert = false;

    //set the arrays 4 distance 
    public static Double[] karrayDistance = {

    }; 

    //set the arrays 4 velocity
    public static Double[] karrayVelocity = {
        10.2, //0
        298.2, //1
        120.3,  //...
    };

    //set the arrays 4 angles 
    public static Double[] karrayAngles = {

    };

    //set the arrays 4 the launch heights  
    public static Double[] karrayLaunchHeight = {

    };
   
    //static final Gains kGains = new Gains(0.15, 0.0, 1, 0, 0, 1);

}