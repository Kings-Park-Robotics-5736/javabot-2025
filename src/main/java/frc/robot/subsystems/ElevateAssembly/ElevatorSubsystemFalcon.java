package frc.robot.subsystems.ElevateAssembly;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.Limits;


public class ElevatorSubsystemFalcon extends SubsystemBase {

    private final TalonFX m_follower;
    private final TalonFX m_leader;

    private TalonFXConfiguration configs;
    private ElevatorFeedforward m_feedforward;
    private final String m_name;

    private Limits m_limits;

    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;

    private int staleCounter = 0;
    private double lastPosition = 0;

    private double m_setpoint;
    private boolean manualControl;
    private final VoltageOut m_sysidControl = new VoltageOut(0);


     /********************************************************
     * SysId variables
     ********************************************************/
   
    private final SysIdRoutine m_sysIdRoutine;
    private final PositionVoltage m_poositionVoltage = new PositionVoltage(0);

    public ElevatorSubsystemFalcon( String _name) {

        m_leader = new TalonFX(ElevatorConstants.kLeaderDeviceId, ElevatorConstants.kCanName);
        m_follower = new TalonFX(ElevatorConstants.kFollowerDeviceId, ElevatorConstants.kCanName);
       
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = ArmConstants.kPidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = ArmConstants.kPidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = ArmConstants.kPidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator LEADER. Restart robot!");
        }

        if (!TalonUtils.ApplyTalonConfig(m_follower, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator FOLLOWER. Restart robot!");
        }

        m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
        m_leader.setNeutralMode(NeutralModeValue.Brake);
        m_follower.setNeutralMode(NeutralModeValue.Brake);
        


        m_limits = ElevatorConstants.kLimits;
        m_name = _name;
        manualControl = true;

        m_feedforward = new ElevatorFeedforward(ElevatorConstants.kFFValues.ks, ElevatorConstants.kFFValues.kg, ElevatorConstants.kFFValues.kv, ElevatorConstants.kFFValues.ka);

        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(null, Volts.of(2), null, state->SignalLogger.writeString("arm-state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        m_leader.setControl(m_sysidControl.withOutput(volts));
                    },
                 null,
                    this)); 

        BaseStatusSignal.setUpdateFrequencyForAll(250,m_leader.getPosition(), m_leader.getVelocity(), m_leader.getMotorVoltage());
        m_leader.optimizeBusUtilization();
       
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
    }

    public void stopElevator() {
        setSpeed(0);
    }


    public Boolean IsElevatorUp() {
        return getElevatorPosition() > m_limits.high-3;
    }

    public Boolean IsElevatorDown() {
        return getElevatorPosition() < m_limits.low+3;
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
        return new FunctionalCommand(
                () -> {manualControl = false; InitMotionProfile(position);},
                () -> {},
                (interrupted) -> stopElevator(),
                () -> isFinished(), this);
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
        return m_leader.getVelocity().refresh().getValueAsDouble() / 60;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); 
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
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        m_setpoint = setpoint;
        m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getElevatorPosition(), 0);
    }

    /**
     * @brief Checks if the elevator has reached its target
     * @return true if the elevator has reached its target, false otherwise
     */
    private Boolean ElevatorReachedTarget() {

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
        return delta < ElevatorConstants.kDiffThreshold || (delta < ElevatorConstants.kStaleTolerance && staleCounter > ElevatorConstants.kStaleThreshold);
    }

    private Boolean isFinished() {
        var isFinished =  ElevatorReachedTarget();
        //SmartDashboard.putBoolean("isfinished " + m_name, isFinished);
        return isFinished;
    }

    /**
     * @brief Runs the Elevator to a given position in absolute rotations
     * 
     */
    private void RunElevator() {

        m_prior_iteration_setpoint = m_profile.calculate(0.02, m_prior_iteration_setpoint,new TrapezoidProfile.State(m_setpoint, 0));

        double ff = m_feedforward.calculate(m_prior_iteration_setpoint.position, m_prior_iteration_setpoint.velocity);

        m_leader.setControl(m_poositionVoltage.withFeedForward(ff).withPosition(m_prior_iteration_setpoint.position));

    }

}