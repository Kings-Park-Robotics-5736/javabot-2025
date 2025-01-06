package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;

public class ClimbSubsystem extends SubsystemBase {

    private ClimbMotorSubsystem left;
    private ClimbServoSubsystem servo;

    public ClimbSubsystem() {

        left = new ClimbMotorSubsystem(Constants.LeftClimbConstants.kPidValues,
                Constants.LeftClimbConstants.kFFValues, Constants.LeftClimbConstants.kDeviceId, "Left",true);

        
        servo = new ClimbServoSubsystem();

    }

    @Override
    public void periodic() {

    }


    public Command RunClimbForwardCommand() {
        return left.RunClimbForwardCommand();
    }

    public Command RunClimbBackwardCommand() {
        return left.RunClimbBackwardCommand();
    }

// hutch wants an option to run the climb independly (possibly)
    public Command RunLeftClimbForwardCommand() {
        return left.RunClimbForwardCommand();
    }


    public Command RunLeftClimbBackwardCommand() {
        return left.RunClimbBackwardCommand();
    }


    public Command ReleaseClimb(){
        return servo.ClimbServoOpenCommand();
    }

     public Command CloseClimb(){
        return servo.ClimbServoCloseCommand();
    }



    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, PositionType whichClimb) {

            return left.sysIdQuasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, PositionType whichClimb) {
       
            return left.sysIdDynamic(direction);
    }

}
