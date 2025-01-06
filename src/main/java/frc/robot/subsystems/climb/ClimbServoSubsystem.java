package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
public class ClimbServoSubsystem extends SubsystemBase {
    Servo climbServo = new Servo(0);
    Servo climbServoSmall = new Servo(1);

    public ClimbServoSubsystem(){
        climbServo.setAngle(70);
        climbServoSmall.setAngle(180);
    }


    
        public Command ClimbServoOpenCommand() {
        return new FunctionalCommand(
                () -> { 
                    System.out.println("-----------------Starting Climb Servo to Position 90 --------------");
                },
                () -> {climbServo.setAngle(160);
                    climbServoSmall.setAngle(10);
                    System.out.println(climbServo.getAngle());
                    },
                (interrupted) -> {},
                () -> false, this);
    }

        public Command ClimbServoCloseCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Climb Servo to position 0--------------");
                },
                () -> {climbServo.setAngle(70);
                climbServoSmall.setAngle(180);},
                (interrupted) -> {},
                () -> false, this);
    
    }    




}
    
    

