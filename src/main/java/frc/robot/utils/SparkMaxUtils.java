package frc.robot.utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;

public class SparkMaxUtils {
     /*
         * Apply the configuration to the SPARK MAX.
         *
         * kResetSafeParameters is used to get the SPARK MAX to a known state. This
         * is useful in case the SPARK MAX is replaced.
         *
         * kPersistParameters is used to ensure the configuration is not lost when
         * the SPARK MAX loses power. This is useful for power cycles that may occur
         * mid-operation.
         */
    public static boolean ApplySparkMaxConfig(SparkMax motor, SparkMaxConfig config) {
        for (int i = 0; i < 5; i++) {
            try {
                if (motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) == REVLibError.kOk) {
                    return true;
                }
            } catch (Exception e) {
                System.out.println("Motor Initialization Failed. Trying again...");
            }
        }
        return false;
   }

}
