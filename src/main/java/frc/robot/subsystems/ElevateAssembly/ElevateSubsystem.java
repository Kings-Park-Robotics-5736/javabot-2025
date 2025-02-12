package frc.robot.subsystems.ElevateAssembly;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final EndeffectorSubsystem m_endeffector;
    private final ElevatorSubsystemFalcon m_elevator;

    public ElevateSubsystem(){
        m_arm = new ArmSubsystemFalcon();
        m_endeffector = new EndeffectorSubsystem();
        m_elevator = new ElevatorSubsystemFalcon("Elevator");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Forward Limit Reached", m_endeffector.ForwardLimitReached());
        SmartDashboard.putBoolean("Reverse Limit Reached", m_endeffector.ReverseLimitReached());
        
    }

    public void setArmInitialPosition(){
        m_arm.setInitialPosition(getArmEncoderPosition());
    }

    public double getArmEncoderPosition(){
        return m_endeffector.getArmEncoder();
    }

    //TODO: Make commands for the arm and elevator to go to each named position


    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdQuasistatic(direction);
    }

    public Command sysIdArmDynamic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdDynamic(direction);
    }

    public Command sysIdElevatorQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevator.sysIdQuasistatic(direction);
    }

    public Command sysIdElevatorDynamic(SysIdRoutine.Direction direction) {
        return m_elevator.sysIdDynamic(direction);
    }

    public Command RunArmToPositionCommand(double position){
        return m_arm.RunArmToPositionCommand(position);
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmUpManualSpeedCommand(speed);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmDownManualSpeedCommand(speed);
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier speed){
        return m_elevator.RunElevatorManualSpeedCommand(speed);
    }

    public Command RunElevatorToPositionCommand(double position){
        return m_elevator.RunElevatorToPositionCommand(position);
    }
    public Command ResetElevatorEncoder(){
        return m_elevator.ResetElevatorEncoderCommand();
    }

    //ensure arm is out of the way of the intake cone before rotating
    public Command GoOutOfTheWay(){
        return (RunElevatorToPositionCommand(ElevatorConstants.kOutofthewayPosition).unless(()->m_elevator.elevatorInSafeSpot()));
    }

    public Command GoToIntakeAndIntake(){
        return GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())));
    }


    private Command GoToScorePosition(double elevatorPosition, double armFinalPosition){
        return GoOutOfTheWay().andThen(RunElevatorToPositionCommand(elevatorPosition).alongWith(m_arm.RunArmToPositionCommand(armFinalPosition).alongWith(Regrip())));
    }

    private Command Score(double elevatorPosition, double armFinalPosition){
        return GoToScorePosition(elevatorPosition, armFinalPosition).andThen(m_endeffector.Score(false));
    }

    private Command GoToL4Position(double elevatorPosition, double armPrepPosition, double armFinalPosition){
        return GoOutOfTheWay().andThen(RunElevatorToPositionCommand(elevatorPosition).alongWith(Regrip()).alongWith(m_arm.RunArmToPositionCommand(armPrepPosition)))
        .andThen(m_arm.RunArmToPositionCommand(armFinalPosition));
    }

    private Command ScoreL4(double elevatorPosition, double armPrepPosition, double armFinalPosition){
        return GoToL4Position(elevatorPosition, armPrepPosition, armFinalPosition).andThen(m_endeffector.Score(true)).andThen(PrepForIntakePosition());
    }

    //Commands to go to AND shoot out for all levels
    public Command ScoreL2Command(){
        return Score(ElevatorConstants.kL2Position, ArmConstants.L2Angle);
    }
    public Command ScoreL3Command(){
        return Score(ElevatorConstants.kL3Position, ArmConstants.L3Angle);
    }
    public Command ScoreL4Command(){
        return ScoreL4(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle);
    }

    //Command to shoot out for L1-L3
    public Command OnlyScore(){
        return m_endeffector.Score(false).andThen(PrepForIntakePosition());
    }

    //commands to go to the scoring commands, but NOT shoot
    public Command GotoScoreL2PositionCommand(){
        return GoToScorePosition(ElevatorConstants.kL2Position, ArmConstants.L2Angle);
    }
    public Command GotoScoreL3PositionCommand(){
        return GoToScorePosition(ElevatorConstants.kL3Position, ArmConstants.L3Angle);
    }
    public Command GotoScoreL4PositionCommand(){
        return GoToL4Position(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle);
    }


    public Command PrepScoreL4(){
        return GoOutOfTheWay().andThen( m_arm.RunArmToPositionCommand(ArmConstants.vertical));
    }

    public Command PrepForIntakePosition(){
        return GoOutOfTheWay().andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle).alongWith(RunElevatorToPositionCommand(ElevatorConstants.kIntakeWaitingPosition)));
    }

    public Command Regrip(){
        return m_endeffector.ReGrip();
    }

    
    
}
