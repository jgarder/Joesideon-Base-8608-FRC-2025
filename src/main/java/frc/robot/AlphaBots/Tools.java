package frc.robot.AlphaBots;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Tools {
    

  public static boolean isPosAtSetpoint(double currentPos,double setpointPos,double tolerance) {
      var difference = currentPos - setpointPos;
      if(tolerance > Math.abs(difference)){
      return true;
      }else {return false;}
      //we could also just use ->return MathUtil.isNear(setpointPos, currentPos, tolerance);
  }

  public static StatusCode SetConfigToTalonFX(TalonFX TalonFX, TalonFXConfiguration Config, String Classname) {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      //PUT MOTORS TO BE CONFIGED HERE
      status = TalonFX.getConfigurator().apply(Config,10);
      //
      if (status.isOK()) return status;
    }
    if(!status.isOK()) {
      System.out.println(Classname + " Could not apply configs, error code: " + status.toString());
      
    }
    return status;
  }

  //I put this in here because i wanted to add something with the alphabots library
  public static double getExpoJoystickInput(double getLeftAxis, double MaxSpeed){
    //Y for X, X for Y, because FRC? //this doesn't make sense anymore because of code optimizations
    double output = ((0.2*getLeftAxis)+(0.8*Math.pow(getLeftAxis, 3))) * MaxSpeed;
    return -output;
  }
}
