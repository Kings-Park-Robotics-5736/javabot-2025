package frc.robot.subsystems.ElevateAssembly;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.ScoringPositionSelector;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final EndeffectorSubsystem m_endeffector;
    private final ElevatorSubsystemFalcon m_elevator;
    private DigitalInput m_coralIntakeSensor;
    private final Trigger coralSensorIntakeTrigger;
    private final ScoringPositionSelector m_ScoringPositionSelector;
    private final DriveSubsystem m_robotDrive;

    private Boolean m_isAuto = false;


    public ElevateSubsystem(ScoringPositionSelector scoringPositionSelector, DriveSubsystem robotDrive){
        m_arm = new ArmSubsystemFalcon(()->getArmEncoderPosition());
        m_endeffector = new EndeffectorSubsystem();
        m_elevator = new ElevatorSubsystemFalcon("Elevator");
        m_coralIntakeSensor = new DigitalInput(0);
        m_ScoringPositionSelector = scoringPositionSelector;
        m_robotDrive = robotDrive;

        SmartDashboard.putData(m_arm);
        SmartDashboard.putData(m_elevator);
        SmartDashboard.putData(m_endeffector);
        SmartDashboard.putData(m_robotDrive);


        coralSensorIntakeTrigger  =  new Trigger(() -> {
                        return !m_isAuto && coralOnCone() && ! m_endeffector.ForwardLimitReached() && ! m_endeffector.ReverseLimitReached();
                }).onTrue(((new WaitCommand(0.5).andThen(GoToIntakeAndIntake()))));

    }

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("Forward Limit Reached", m_endeffector.ForwardLimitReached());
        //SmartDashboard.putBoolean("Reverse Limit Reached", m_endeffector.ReverseLimitReached());
        
        SmartDashboard.putString("MANUAL SELECTED POSITION", m_ScoringPositionSelector.getScorePositionString());
        
        
    }


    public void setIsAutonomous(boolean isAuto){
        m_isAuto = isAuto;
    }

    public boolean coralOnCone(){
        return !m_coralIntakeSensor.get();
    }

    public void setArmInitialPosition(){
        m_arm.setInitialPosition(getArmEncoderPosition());
    }

    public double getArmEncoderPosition(){
        return m_endeffector.getArmEncoder();
    }

    public void resetArmElevatorAfterDisable() {
        m_arm.resetAfterDisable();
        m_elevator.resetAfterDisable();
    }

    public Command ResetElevatorEncoder(){
        return m_elevator.ResetElevatorEncoderCommand();
    }


    /*************************************************
     * SysId Commands
     *************************************************/

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



    /***********************************************
     * Commands to run the arm and elevator manually
     ***********************************************/

    public Command RunArmUpManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmUpManualSpeedCommand(speed);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier speed){
        return m_arm.RunArmDownManualSpeedCommand(speed);
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier speed){
        return m_elevator.RunElevatorManualSpeedCommand(speed);
    }


    public Command RunArmToPositionCommand(double position){
        return m_arm.RunArmToPositionCommand(position);
    }

    public Command RunElevatorToPositionCommand(double position){
        return m_elevator.RunElevatorToPositionCommand(position);
    }
   



    //ensure arm is out of the way of the intake cone before rotating
    public Command GoOutOfTheWay(){
        return (RunElevatorToPositionCommand(ElevatorConstants.kOutofthewayPosition).unless(()->m_elevator.elevatorInSafeSpot())).withName("GoOutOfTheWay");
    }

    /**********************************************
     * Intake Commands
     **********************************************/
    public Command GoToIntakeAndIntake(){
        return (GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle))
        .andThen(m_elevator.RunElevatorToIntakeSafeCommandRetries().alongWith(m_endeffector.Intake()))
        .andThen(new WaitUntilCommand(()->MathUtils.IsAwayFromIntakeStation(m_robotDrive.getPose())))
        .andThen(PrepScore())).withName("GoToIntakeAndIntake");
    }

    public Command GoToIntakeAndIntakeNoWait(){
        return (GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle))
        .andThen(m_elevator.RunElevatorToIntakeSafeCommandRetries().alongWith(m_endeffector.Intake()))
        .andThen(PrepScore())).withName("GoToIntakeAndIntakeNoWait");
    }


    public Command GoToIntakeAndIntakeAsDriveWithChecks(){
        return ((GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())))).unless(()->!coralOnCone())
        .andThen(new WaitUntilCommand(()->MathUtils.IsAwayFromIntakeStation(m_robotDrive.getPose())))
        .andThen(PrepScore())).withName("GoToIntakeAndIntakeAsDriveWithChecks");
    }


    public Command GoToIntakeAndIntakeAsDriveWithChecksNoWallWait(){
        return ((GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())))).unless(()->!coralOnCone())).withName("GoToIntakeAndIntakeAsDriveWithChecksNoWallWait");
    }

    public Command WaitForCoral(){
        return new WaitUntilCommand(()->coralOnCone()).withName("WaitForCoral");
    }

    public Command PrepForIntakePosition(){
        return (GoOutOfTheWay().andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle).alongWith(RunElevatorToPositionCommand(ElevatorConstants.kIntakeWaitingPosition)))).withName("PrepForIntakePosition");
    }



    // Bring the arm and the elevator up to the scoring position for l1-l3
    private Command GoToScorePosition(double elevatorPosition, double armFinalPosition){
        return (GoOutOfTheWay().andThen(RunElevatorToPositionCommand(elevatorPosition).alongWith(m_arm.RunArmToPositionCommand(armFinalPosition).alongWith(ReGripL2L3())))).withName("GoToScorePosition");
    }

    //bring the arm and the elevator up the scoring position and score for l1-l3
    private Command Score(double elevatorPosition, double armFinalPosition){
        return (GoToScorePosition(elevatorPosition, armFinalPosition).andThen(m_endeffector.Score(false))).withName("Score");
    }

    // bring the arm and the elevator up to the scoring position for l4
    private Command GoToL4Position(double elevatorPosition, double armPrepPosition, double armFinalPosition){
        return (GoOutOfTheWay().andThen(RunElevatorToPositionCommand(elevatorPosition).alongWith(Regrip()).alongWith(m_arm.RunArmToPositionCommand(armPrepPosition, false)))
        .andThen(m_arm.RunArmToPositionCommand(armFinalPosition, false))).withName("GoToL4Position");
    }

    //bring the arm and the elevator up to the scoring position and score for l4
    private Command ScoreL4(double elevatorPosition, double armPrepPosition, double armFinalPosition){
        return (GoToL4Position(elevatorPosition, armPrepPosition, armFinalPosition).andThen(m_endeffector.Score(true)).andThen(new WaitCommand(.25)).andThen(PrepForIntakePosition())).withName("ScoreL4");
    }

    /**********************************
     * Commands to go to and shoot at all levels in one go
     * *******************************/
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
        return (m_endeffector.Score(false).andThen(new WaitCommand(.25)).andThen(PrepForIntakePosition())).withName("OnlyScore");
    }

    //command to shoot out for L4
    public Command OnlyScoreL4(){
        return (m_endeffector.Score(true).andThen(new WaitCommand(.25)).andThen(PrepForIntakePosition())).withName("OnlyScoreL4");
    }
    public Command OnlyScoreL4NoIntakeReturn(){
        return m_endeffector.Score(true);
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


    public Command PrepScore(){
        return (GoOutOfTheWay().andThen( m_arm.RunArmToPositionCommand(ArmConstants.AllHoldingAngle))).withName("PrepScore");
    }


    public Command Regrip(){
        return m_endeffector.ReGrip();
    }

    public Command ReGripL2L3(){
        return m_endeffector.ReGripL2L3();
    }


    public Command ClearAlgaeHighStep1(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeHighPosition1).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.ClearAlgaeAngle));
    }

    public Command ClearAlgaeLowStep1(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeLowPosition1).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.ClearAlgaeAngle));
    }

    public Command ClearAlgaeHighStep2(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeHighPosition2);
    }

    public Command ClearAlgaeLowStep2(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeLowPosition2);
    }


    
   


    /***************************************
     * Compound commands for moving and such
     ***************************************/

     public Command GetPositionCommandFromHeight(ScoreHeight height){
        switch(height){
            case L2:
                return GotoScoreL2PositionCommand();
            case L3:
                return GotoScoreL3PositionCommand();
            case L4:
            default:
                return GotoScoreL4PositionCommand();
        }
     }

     public Command AutoIntakeAndL4PositionWhileDriving(){
        return 
         m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake()))
        .andThen(GotoScoreL4PositionCommand());
     }

     

     public Command DriveAndScoreCommand(Command initialCommand, DriveSubsystem robotDrive, Boolean left, ScoreHeight height){
        return (initialCommand.alongWith( 
            GoToIntakeAndIntakeAsDriveWithChecks().andThen(
            new WaitUntilCommand(()->MathUtils.IsWithinRange(robotDrive.getPose())))
            .andThen(GetPositionCommandFromHeight(height)))
            .andThen(() -> robotDrive.forceStop())
        .andThen(height == ScoreHeight.L4 ? OnlyScoreL4() : OnlyScore())).withName("DriveAndScoreCommand");
        
     }


     public Command DriveToNearestScoreCommand(DriveSubsystem robotDrive, Boolean left, ScoreHeight height){
        return DriveAndScoreCommand(TrajectoryCommandsFactory.getScoringClosestCommand(robotDrive, left, height == ScoreHeight.L4), robotDrive, left, height);
        
     }

     public Command DriveToSelectedCommand(DriveSubsystem robotDrive, Boolean left, ScoreHeight height){
        return DriveAndScoreCommand(TrajectoryCommandsFactory.getScoringSelectedCommand(robotDrive, false, height == ScoreHeight.L4, ()-> MathUtils.BuildMapKeyString(m_ScoringPositionSelector.getScorePosition(), left, height== ScoreHeight.L4)), robotDrive, left, height);
        
     }


     public Command DriveToClearingPosition(DriveSubsystem robotDrive){
        return (TrajectoryCommandsFactory.getClearingSelectedCommand(robotDrive, ()-> MathUtils.BuildMapKeyStringClear(m_ScoringPositionSelector.getScorePosition())));
     }
     

     public Command ClearAlgaeHigh(DriveSubsystem robotDrive){
        return (DriveToClearingPosition(robotDrive).alongWith(ClearAlgaeHighStep1()).andThen(ClearAlgaeHighStep2()));
     }

        public Command ClearAlgaeLow(DriveSubsystem robotDrive){
            return (DriveToClearingPosition(robotDrive).alongWith(ClearAlgaeLowStep1()).andThen(ClearAlgaeLowStep2()));
        }
    
    
}
