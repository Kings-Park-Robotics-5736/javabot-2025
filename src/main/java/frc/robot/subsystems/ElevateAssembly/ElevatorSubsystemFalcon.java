package frc.robot.subsystems.ElevateAssembly;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.Limits;


public class ElevatorSubsystemFalcon extends SubsystemBase {

    private final TalonFX m_follower;
    private final TalonFX m_leader;

    private TalonFXConfiguration configs;
    private final String m_name;

    private Limits m_limits;


    private int staleCounter = 0;
    private double lastPosition = 0;
    private int stallCounter = 0;

    private double m_setpoint;
    private boolean manualControl;
    private boolean stallStop = false;
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

 private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));

     /********************************************************
     * SysId variables
     ********************************************************/
   
    private final SysIdRoutine m_sysIdRoutine;


    public ElevatorSubsystemFalcon( String _name) {

        m_leader = new TalonFX(ElevatorConstants.kLeaderDeviceId, ElevatorConstants.kCanName);
        m_follower = new TalonFX(ElevatorConstants.kFollowerDeviceId, ElevatorConstants.kCanName);
       
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = ElevatorConstants.kPidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = ElevatorConstants.kPidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = ElevatorConstants.kPidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
         configs.Slot0.kA = ElevatorConstants.kFFValues.ka;
        configs.Slot0.kG = ElevatorConstants.kFFValues.kg;
        configs.Slot0.kV = ElevatorConstants.kFFValues.kv;
        configs.Slot0.kS = ElevatorConstants.kFFValues.ks;
        configs.Voltage.PeakForwardVoltage = 6;
        configs.Voltage.PeakReverseVoltage = -6;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMaxVelocity;// 2* Math.PI;

        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kMaxAcceleration; //4* Math.PI; 
        configs.MotionMagic.MotionMagicJerk = ElevatorConstants.kMaxJerk; //20*Math.PI; 

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator LEADER. Restart robot!");
        }

        if (!TalonUtils.ApplyTalonConfig(m_follower, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator FOLLOWER. Restart robot!");
        }

        m_follower.setControl(new Follower(m_leader.getDeviceID(), false));

        m_limits = ElevatorConstants.kLimits;
        m_name = _name;
        manualControl = true;

        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(null, Volts.of(2), null),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        // CANNOT use set voltage, it does not work. This normalizes the voltage between
                        // -1 and 0 and 1
                        m_leader.set(volts.in(Volts) / RobotController.getBatteryVoltage());
                    },
                    log -> {
                        log.motor(("ElevatorD"))
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_distance.mut_replace(m_leader.getPosition().refresh().getValueAsDouble(),
                                        Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(m_leader.getVelocity().refresh().getValueAsDouble(),
                                                RotationsPerSecond));

                    },
                    this));

       
       
    }

    @Override
    public void periodic() {

        //optional code to tune pids from smart dashboard
       /* double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        if((p != m_pidValues.p)) { m_pidController.setP(p); m_pidValues.p = p; }
        if((i != m_pidValues.i)) { m_pidController.setI(i); m_pidValues.i = i; }
        if((d != m_pidValues.d)) { m_pidController.setD(d); m_pidValues.d = d; }
        */

        if(!manualControl){
            RunElevator();
           
        }
        //SmartDashboard.putNumber("Stator Current", m_leader.getStatorCurrent().refresh().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Enc Pos", getElevatorPosition());
        

    }

    public void stopElevator() {
        setSpeed(0);
    }

    public void resetAfterDisable(){
        System.out.println("Reset ARM After Disable!!!");
        stopElevator();
        manualControl = true;
    }


    public Boolean IsElevatorUp() {
        return getElevatorPosition() > m_limits.high-3;
    }

    public Boolean IsElevatorDown() {
        return getElevatorPosition() < m_limits.low+3;
    }

    public Boolean elevatorInSafeSpot(){
        return getElevatorPosition() > ElevatorConstants.kOutofthewayPosition;
    }


    private void PrettyPrint(double position){
        if (position == ElevatorConstants.kIntakePosition){
            System.out.println("==========Begin setting Elevator to Intake position===============");
        }else if (position == ElevatorConstants.kL1Position){
            System.out.println("==========Begin setting Elevator to L1 position===============");
        }else if (position == ElevatorConstants.kL2Position){
            System.out.println("==========Begin setting Elevator to L2 position===============");
        }else if (position == ElevatorConstants.kL3Position){
            System.out.println("==========Begin setting Elevator to L3 position===============");
        }else if (position == ElevatorConstants.kL4Position){
            System.out.println("==========Begin setting Elevator to L4 position===============");
        }else if (position == ElevatorConstants.kOutofthewayPosition){
            System.out.println("==========Begin setting Elevator to Out of the way position===============");
        }else if (position == ElevatorConstants.kIntakeWaitingPosition){
            System.out.println("==========Begin setting Elevator to Intake Waiting position===============");
        }else{
            System.out.println("==========Begin setting Elevator to " + position + "===============");
        }


    }

    /**
     * @brief Runs the Elevator at a given speed (-1 to 1) in manual mode until interrupted
     * @param getSpeed a lambda that takes no arguments and returns the desired speed of the Elevator [ () => double ]
     * @return the composed command to manually drive the Elevator
     */
    public Command RunElevatorManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {manualControl = true;},
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> stopElevator(),
                () -> false, this);
    }

    /**
     * @brief Runs the Elevator to a given position in absolute rotations
     * @param position absolute position to run to (not a lambda)
     * @return the composed command to run the Elevator to a given positions
     */
    public Command RunElevatorToPositionCommand(double position) {
        return (new FunctionalCommand(
                () -> {manualControl = false; InitMotionProfile(position);PrettyPrint(position);},
                () -> {},
                (interrupted) -> {},
                () -> isFinished(true), this)).withName("RunElevatorToPositionCommand");
    }

    public Command RunElevatorToPositionCommandEarlyFinish(double position) {
        return (new FunctionalCommand(
                () -> {manualControl = false; InitMotionProfile(position);PrettyPrint(position);},
                () -> {},
                (interrupted) -> {},
                () -> isFinishedEarly(true), this)).withName("RunElevatorToPositionCommand");
    }


    private Command RunElevatorToIntakeSafeCommand(){
        return (new FunctionalCommand(
                () -> {manualControl = false; InitMotionProfile(ElevatorConstants.kIntakePosition);System.out.println("ELEVATOR TO SAFE INTAKE====================="); stallStop=false;},
                () -> { if (isStalled()  || isFinished(false) ){
                    stallStop = true;
                }
            },
                (interrupted) -> {},
                () -> stallStop, this)).withName("RunElevatorToIntakeSafeCommand");
    }

    public Command RunElevatorToIntakeSafeCommandRetries(){
        return (RunElevatorToIntakeSafeCommand()
                .andThen(((RunElevatorToPositionCommand(ElevatorConstants.kL3Position))
                .andThen(new WaitCommand(0.5))
                .andThen(RunElevatorToIntakeSafeCommand())).unless(()->!isStalled()))).withName("RunElevatorToIntakeSafeCommandRetries");

    }



    public Command ResetElevatorEncoderCommand() {
        return this.runOnce(() -> resetEncoder());
    }

    private void resetEncoder() {
        m_leader.setPosition(0);
    }

    public double getElevatorPosition(){
        return m_leader.getPosition().refresh().getValueAsDouble();
    }

    public double getRotationsPerSecond(){
        return m_leader.getVelocity().refresh().getValueAsDouble();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); 
    }

    public Boolean isStalled(){
        if(m_leader.getStatorCurrent().refresh().getValueAsDouble()>20 || staleCounter > 25){
            stallCounter++;
        }else{
            stallCounter=0;
        }
        if(stallCounter > 25){
            System.out.println("STALL!!!!!!!!!!!!!!!!!!!!!!!");
            return true;
        }
        return false;
    }

  
    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_leader.set(speed);
        SmartDashboard.putNumber("Elevator Position" + m_name, getElevatorPosition());
        SmartDashboard.putNumber("Elevator Speed" + m_name, speed);
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        m_setpoint = setpoint;
        staleCounter = 0;
      
    }

    /**
     * @brief Checks if the elevator has reached its target
     * @return true if the elevator has reached its target, false otherwise
     */
    private Boolean ElevatorReachedTarget(Boolean useStale) {

        // check if the elevator has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(getElevatorPosition() - lastPosition) < ElevatorConstants.kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = getElevatorPosition();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(getElevatorPosition() - m_setpoint);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < ElevatorConstants.kDiffThreshold || (useStale && delta < ElevatorConstants.kStaleTolerance && staleCounter > ElevatorConstants.kStaleThreshold);
    }

    private Boolean isFinished(Boolean useStale) {
        var isFinished =  ElevatorReachedTarget(useStale);
        //SmartDashboard.putBoolean("isfinished " + m_name, isFinished);
        return isFinished;
    }

    private Boolean isFinishedEarly(Boolean useStale) {
        double delta = Math.abs(getElevatorPosition() - m_setpoint);
        var isFinished =  ElevatorReachedTarget(useStale);
        return isFinished || delta < 3;


    }

    /**
     * @brief Runs the Elevator to a given position in absolute rotations
     * 
     */
    private void RunElevator() {

       
        SmartDashboard.putNumber("Elevator m_setpoint" + m_name, m_setpoint);

        m_leader.setControl(m_request.withPosition(m_setpoint));

    }

}