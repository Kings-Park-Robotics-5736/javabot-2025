package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;

public class IntakeSubsystem extends SubsystemBase {

    private IntakeRollersSubsystem top;
    private IntakeRollersSubsystem bottom;

    public IntakeSubsystem() {

        top = new IntakeRollersSubsystem(Constants.UpperIntakeConstants.kPidValues,
                Constants.UpperIntakeConstants.kFFValues, Constants.UpperIntakeConstants.kDeviceId, "Top");

        bottom = new IntakeRollersSubsystem(Constants.LowerIntakeConstants.kPidValues,
                Constants.LowerIntakeConstants.kFFValues, Constants.LowerIntakeConstants.kDeviceId, "Bottom");

    }

    @Override
    public void periodic() {

    }


    public Command RunIntakeForwardCommand() {
        return Commands.parallel(top.RunIntakeForwardCommand(), bottom.RunIntakeForwardCommand());
    }

    public Command RunIntakeBackwardCommand() {
        return Commands.parallel(top.RunIntakeBackwardCommand(), bottom.RunIntakeBackwardCommand());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, PositionType whichIntake) {
        if (whichIntake == PositionType.TOP) {
            return top.sysIdQuasistatic(direction);
        } else {
            return bottom.sysIdQuasistatic(direction);
        }

    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, PositionType whichIntake) {
         if (whichIntake == PositionType.TOP) {
            return top.sysIdDynamic(direction);
        } else {
            return bottom.sysIdDynamic(direction);
        }
    }

}
