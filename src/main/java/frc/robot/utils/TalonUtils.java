package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonUtils {
     public static boolean ApplyTalonConfig(TalonFX motor, TalonFXConfiguration config){
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
            else {
                System.out.println("Motor Initialization Failed. Trying again...");
            }

        }
        return status.isOK();
        
    }

    public static boolean ApplyCanCoderConfig(CANcoder cancoder, CANcoderConfiguration config){
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = cancoder.getConfigurator().apply(config);
            if (status.isOK())
                break;
            else {
                System.out.println("Cancoder Initialization Failed. Trying again...");
            }

        }
        return status.isOK();
        
    }

    public static int getSpeedRotationsPerMinute(TalonFX motor) {
        return (int)(motor.getVelocity().refresh().getValueAsDouble() * 60);
    }
    
}
