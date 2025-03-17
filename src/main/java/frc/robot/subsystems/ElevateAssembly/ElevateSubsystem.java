package frc.robot.subsystems.ElevateAssembly;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.field.ScoringPositions.ScoreHeight;
import frc.robot.field.ScoringPositions.ScorePositions;
import frc.robot.subsystems.LEDSubsystem;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.ScoringPositionSelector;
import frc.robot.utils.Types.LEDState;

public class ElevateSubsystem extends SubsystemBase {
    private final ArmSubsystemFalcon m_arm;
    private final EndeffectorSubsystem m_endeffector;
    private final ElevatorSubsystemFalcon m_elevator;
    private DigitalInput m_coralIntakeSensor;
    private DigitalInput m_chuteSensor;
    private final ScoringPositionSelector m_ScoringPositionSelector;
    private final DriveSubsystem m_robotDrive;
    private final LEDSubsystem m_ledSystem;
    private int m_cagePosition;

    private Boolean m_isAuto = false;
    private Boolean isAutoDrive = false;
    private Boolean isChuteSeen = false;
    private Boolean isClimbing = false;


    public ElevateSubsystem(ScoringPositionSelector scoringPositionSelector, DriveSubsystem robotDrive, LEDSubsystem ledSubsystem) {
        m_arm = new ArmSubsystemFalcon(()->getArmEncoderPosition());
        m_endeffector = new EndeffectorSubsystem();
        m_elevator = new ElevatorSubsystemFalcon("Elevator");
        m_coralIntakeSensor = new DigitalInput(0);
        m_chuteSensor = new DigitalInput(1);

        m_ScoringPositionSelector = scoringPositionSelector;
        m_robotDrive = robotDrive;

        m_cagePosition = 0;

        SmartDashboard.putData(m_arm);
        SmartDashboard.putData(m_elevator);
        SmartDashboard.putData(m_endeffector);
        SmartDashboard.putData(m_robotDrive);

        SmartDashboard.putNumber("Cage Position", 1);


        m_ledSystem = ledSubsystem;


        new Trigger(() -> {
                        return !m_isAuto && !isAutoDrive && coralOnCone() && ! m_endeffector.ForwardLimitReached() && ! m_endeffector.ReverseLimitReached();
                }).onTrue(((new WaitCommand(0.25).andThen(GoToIntakeAndIntake()))));


                new Trigger( ()->{
                    return chuteSensorSeen();
                }).onTrue(Commands.runOnce(()->isChuteSeen = true));

                new Trigger(()->{
                    return coralOnCone();
                }).onTrue(Commands.runOnce(()->isChuteSeen = false));

                new Trigger( ()->{
                    return !m_robotDrive.seeReef() && (isClimbing || coralOnCone() || chuteSensorSeen() || m_endeffector.ForwardLimitReached() || m_endeffector.ReverseLimitReached());
                }).onTrue(Commands.runOnce(()->m_ledSystem.SetLEDState(LEDState.HAVE_NOTE), m_ledSystem));


                new Trigger( ()->{
                    return m_robotDrive.seeReef() && (isClimbing || coralOnCone() || chuteSensorSeen() || m_endeffector.ForwardLimitReached() || m_endeffector.ReverseLimitReached());
                }).onTrue(Commands.runOnce(()->m_ledSystem.SetLEDState(LEDState.SEE_TAG), m_ledSystem));


                new Trigger( ()->{
                    return !coralOnCone() && !chuteSensorSeen() && !m_endeffector.ForwardLimitReached() && !m_endeffector.ReverseLimitReached();
                }).onTrue(Commands.runOnce(()->m_ledSystem.SetLEDState(LEDState.NONE), m_ledSystem));

                
                
    }

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("Forward Limit Reached", m_endeffector.ForwardLimitReached());
        //SmartDashboard.putBoolean("Reverse Limit Reached", m_endeffector.ReverseLimitReached());
        
        SmartDashboard.putString("MANUAL SELECTED POSITION", m_ScoringPositionSelector.getScorePositionString());

        m_cagePosition = (int)SmartDashboard.getNumber("Cage Position", 1);
        
    }


    public void setIsClimbing(boolean isClimb){
        isClimbing = isClimb;
    }

    public void setIsAutonomous(boolean isAuto){
        m_isAuto = isAuto;
    }

    public boolean coralOnCone(){
        return !m_coralIntakeSensor.get();
    }

    public boolean chuteSensorSeen(){
        return !m_chuteSensor.get();
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

    public Command RunElevatorToPositionCommand(double position, Boolean async){
        return m_elevator.RunElevatorToPositionCommand(position, async);
    }
   



    //ensure arm is out of the way of the intake cone before rotating
    public Command GoOutOfTheWay(){
        return (m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kOutofthewayPosition).unless(()->(m_elevator.elevatorInSafeSpot() ))).withName("GoOutOfTheWay");
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
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())))).unless(()->!coralOnCone()&&(m_endeffector.ForwardLimitReached() || m_endeffector.ReverseLimitReached())))
        .andThen(new WaitUntilCommand(()->MathUtils.IsAwayFromIntakeStation(m_robotDrive.getPose()))
        .andThen(PrepScore())).withName("GoToIntakeAndIntakeAsDriveWithChecks");
    }


    public Command GoToIntakeAndIntakeAsDriveWithChecksNoWallWait(){
        return ((GoOutOfTheWay()
        .andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake())))).unless(()->!coralOnCone()&&(m_endeffector.ForwardLimitReached() || m_endeffector.ReverseLimitReached()))).withName("GoToIntakeAndIntakeAsDriveWithChecksNoWallWait");
    }

    public Command WaitForCoral(){
        return new WaitUntilCommand(()->coralOnCone()).withName("WaitForCoral");
    }

    public Command WaitForCoralOrChute(){
        return new WaitUntilCommand(()->{ return coralOnCone() || chuteSensorSeen() || isChuteSeen;}).withName("WaitForCoral");
    }

    public Command PrepForIntakePosition(){
        return PrepForIntakePosition(false);
    }

    public Command PrepForIntakePosition(Boolean async){
        return (GoOutOfTheWay().andThen(m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle, async).alongWith(RunElevatorToPositionCommand(ElevatorConstants.kIntakeWaitingPosition, async)))).withName("PrepForIntakePosition");
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
    private Command GoToL4Position(double elevatorPosition, double armPrepPosition, double armFinalPosition, boolean endEarly){
        return (GoOutOfTheWay().andThen(RunElevatorToPositionCommand(elevatorPosition).alongWith(Regrip()).alongWith(m_arm.RunArmToPositionCommand(armPrepPosition, endEarly)))
        .andThen(m_arm.RunArmToPositionCommand(armFinalPosition, endEarly))).withName("GoToL4Position");
    }

    private Command GoToL4PositionNoPre(double elevatorPosition, double armPrepPosition, double armFinalPosition, boolean endEarly){
        return (GoOutOfTheWay().andThen(m_elevator.RunElevatorToPositionCommandEarlyFinish(elevatorPosition).alongWith(Regrip()).alongWith(m_arm.RunArmToPositionCommand(armFinalPosition, endEarly)))
        ).withName("GoToL4PositionNoPre");
    }



    //bring the arm and the elevator up to the scoring position and score for l4
    private Command ScoreL4(double elevatorPosition, double armPrepPosition, double armFinalPosition, boolean endEarly){
        return (GoToL4Position(elevatorPosition, armPrepPosition, armFinalPosition,endEarly).andThen(m_endeffector.Score(true)).andThen(new WaitCommand(.15)).andThen(PrepForIntakePosition())).withName("ScoreL4");
    }

    private Command ScoreL4NoPre(double elevatorPosition, double armPrepPosition, double armFinalPosition, boolean endEarly){
        return (GoToL4PositionNoPre(elevatorPosition, armPrepPosition, armFinalPosition,endEarly).andThen(m_endeffector.Score(true)).andThen(new WaitCommand(.15)).andThen(PrepForIntakePosition(true))).withName("ScoreL4NoPre");
    }

    private Command ScoreL4NoPreNoReturn(double elevatorPosition, double armPrepPosition, double armFinalPosition, boolean endEarly){
        return (GoToL4PositionNoPre(elevatorPosition, armPrepPosition, armFinalPosition,endEarly).andThen(m_endeffector.Score(true))).withName("ScoreL4NoPreNoReturn");
    }



    public Command ElevateHome(){
        return m_arm.RunArmToPositionCommand(ArmConstants.vertical).andThen(m_elevator.RunElevatorToPositionCommand(1));
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
        return ScoreL4(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle, false);
    }

    public Command ScoreL4CommandEarlyEnd(){
        return ScoreL4NoPre(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle, true);
    }

    public Command ScoreL4CommandEarlyEndNoReturn(){
        return ScoreL4NoPreNoReturn(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle, true);
    }


    //Command to shoot out for L1-L3
    public Command OnlyScore(){
        return (m_endeffector.Score(false).andThen(new WaitCommand(.25)).andThen(PrepForIntakePosition(true))).withName("OnlyScore");
    }

    //command to shoot out for L4
    public Command OnlyScoreL4(){
        return (m_endeffector.Score(true).andThen(new WaitCommand(.05)).andThen(PrepForIntakePosition(true))).withName("OnlyScoreL4");
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
        return GotoScoreL4PositionCommand(false);
    }

    public Command GotoScoreL4PositionCommand(Boolean endEarly){
        return GoToL4Position(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle, endEarly);
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
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeHighPosition1).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.vertical));
    }

    public Command ClearAlgaeLowStep1(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeLowPosition1).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.vertical));
    }

    public Command ClearAlgaeHighStep2(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeHighPosition2).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.ClearAlgaeAngle));
    }

    public Command ClearAlgaeLowStep2(){
        return m_elevator.RunElevatorToPositionCommand(ElevatorConstants.kClearAlgaeLowPosition2).alongWith(m_arm.RunArmToPositionCommand(ArmConstants.ClearAlgaeAngle));
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
        new WaitUntilCommand(()->coralOnCone()).andThen(
         m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle))
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake()))
        .andThen(GoToL4Position(ElevatorConstants.kL4Position, ArmConstants.L4PrepAngle, ArmConstants.L4Angle, true)).andThen(m_endeffector.Score(true));
     }


     
     public Command AutoIntakePositionWhileDriving(){
        return 
         m_arm.RunArmToPositionCommand(ArmConstants.intakeAngle)
        .andThen(RunElevatorToPositionCommand(ElevatorConstants.kIntakePosition).alongWith(m_endeffector.Intake()))
        .andThen(PrepScore());
     }


     

     public Command DriveAndScoreCommand(Command initialCommand, DriveSubsystem robotDrive, Boolean left, ScoreHeight height){

        if(height == ScoreHeight.L4){
        return ((Commands.runOnce(()->isAutoDrive=true)
                .andThen(initialCommand.alongWith(GoToIntakeAndIntakeAsDriveWithChecks())
                .andThen(() -> robotDrive.forceStop())
                .andThen(ScoreL4CommandEarlyEnd())))
                .finallyDo((interrupted)->{isAutoDrive = false;}))
                .withName("DriveAndScoreCommandL4");
        }else{
            return (Commands.runOnce(()->isAutoDrive=true)
                    .andThen(initialCommand.alongWith( 
                        GoToIntakeAndIntakeAsDriveWithChecks().andThen(
                        new WaitUntilCommand(()->MathUtils.IsWithinRange(robotDrive.getPose())))
                        .andThen(GetPositionCommandFromHeight(height)))
                    .andThen(() -> robotDrive.forceStop())
                    .andThen(new WaitCommand(.20))
                    .andThen(height == ScoreHeight.L4 ? OnlyScoreL4() : OnlyScore())))
            .finallyDo((interrupted)->{isAutoDrive = false;})
            .withName("DriveAndScoreCommand");
        }
        
     }

     public Command DriveToNearestScoreCommand(DriveSubsystem robotDrive, Boolean left, ScoreHeight height){
        return DriveAndScoreCommand(TrajectoryCommandsFactory.getScoringClosestCommand(robotDrive, left, height == ScoreHeight.L4), robotDrive, left, height);
        
     }

     public Command DriveToSelectedCommand(DriveSubsystem robotDrive, Boolean left, ScoreHeight height){
        return DriveAndScoreCommand(TrajectoryCommandsFactory.getScoringSelectedCommand(robotDrive, false, height == ScoreHeight.L4, ()-> MathUtils.BuildMapKeyString(m_ScoringPositionSelector.getScorePosition(), left, height== ScoreHeight.L4)), robotDrive, left, height);
        
     }

     public Command DriveToSelectedCommand(DriveSubsystem robotDrive, Boolean left, ScoreHeight height,BooleanSupplier noFastClear){
        return DriveAndScoreCommand(TrajectoryCommandsFactory.getScoringSelectedCommand(robotDrive, false, height == ScoreHeight.L4, ()-> MathUtils.BuildMapKeyString(m_ScoringPositionSelector.getScorePosition(), left, height== ScoreHeight.L4)), robotDrive, left, height).andThen(FastClearAlgae(robotDrive).unless(noFastClear));
        
     }


     
     public Command DriveToClearingPosition(DriveSubsystem robotDrive, Boolean fastDrive){
        return (TrajectoryCommandsFactory.getClearingSelectedCommand(robotDrive, fastDrive, ()-> MathUtils.BuildMapKeyStringClear(m_ScoringPositionSelector.getScorePosition())));
     }

     public Command DriveAwayFromClearingPosition(DriveSubsystem robotDrive){
        return (TrajectoryCommandsFactory.getClearingSelectedCommand(robotDrive, false, ()-> MathUtils.BuildMapKeyStringClear(m_ScoringPositionSelector.getScorePosition()) + "REV"));
     }
     


    /**
     * Functions to quickly clear algae when already at the score L4 position.
     */
     public Command FastClearAlgae(DriveSubsystem robotDrive){
        return new ConditionalCommand(ClearAlgaeFastHigh(robotDrive), ClearAlgaeFastLow(robotDrive), ()->isClearHigh());
 
     }

     public Command ClearAlgaeFastHigh(DriveSubsystem robotDrive){
        return ((DriveToClearingPosition(robotDrive, true).andThen(Commands.runOnce(() -> m_robotDrive.forceStop()))).andThen(ClearAlgaeHighStep2())).andThen(DriveAwayFromClearingPosition(robotDrive)).andThen(PrepForIntakePosition());
     }

    public Command ClearAlgaeFastLow(DriveSubsystem robotDrive){
        return ((DriveToClearingPosition(robotDrive, true).andThen(Commands.runOnce(() -> m_robotDrive.forceStop()))).andThen(ClearAlgaeLowStep2())).andThen(DriveAwayFromClearingPosition(robotDrive)).andThen(PrepForIntakePosition());
    }

     public Boolean isClearHigh(){
        return m_ScoringPositionSelector.getScorePosition() == ScorePositions.TWO ||
        m_ScoringPositionSelector.getScorePosition() == ScorePositions.SIX ||
        m_ScoringPositionSelector.getScorePosition() == ScorePositions.TEN;
     }


     public Command ClearAlgaeHigh(DriveSubsystem robotDrive){
        return ((DriveToClearingPosition(robotDrive, false).andThen(Commands.runOnce(() -> m_robotDrive.forceStop()))).alongWith(ClearAlgaeHighStep1()).andThen(ClearAlgaeHighStep2()));
     }

        
        public Command ClearAlgaeLow(DriveSubsystem robotDrive){
            return ((DriveToClearingPosition(robotDrive, false).andThen(Commands.runOnce(() -> m_robotDrive.forceStop()))).alongWith(ClearAlgaeLowStep1()).andThen(ClearAlgaeLowStep2()));
        }



        public Command ClearAlgae(DriveSubsystem robotDrive){
            return new ConditionalCommand(ClearAlgaeHigh(robotDrive), ClearAlgaeLow(robotDrive), ()->isClearHigh());

        }



        public Command DriveToCage(DriveSubsystem robotDrive){
            return Commands.runOnce(()->setIsClimbing(true)).andThen((TrajectoryCommandsFactory.goToSelectedCageCommand(robotDrive, ()->"CAGE"+String.valueOf(m_cagePosition))).alongWith(ElevateHome()));
        }

    
        public Command CagePosRight(){
            return Commands.runOnce(()->{System.out.println("Cage Positon Changed"); if (m_cagePosition == 3 ) m_cagePosition = 1; else m_cagePosition += 1 ;}).andThen(()->SmartDashboard.putNumber("Cage Position", m_cagePosition));
        }
        public Command CagePosLeft(){
            return Commands.runOnce(()->{System.out.println("Cage Positon Changed"); if (m_cagePosition == 1 ) m_cagePosition = 3; else m_cagePosition -= 1 ;}).andThen(()->SmartDashboard.putNumber("Cage Position", m_cagePosition));
        }

    }
