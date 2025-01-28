package frc.robot.subsystems.ElevateAssembly;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax m_follower;
    private final SparkMax m_leader;

    private SparkMaxConfig m_leaderMotorConfig;
    private SparkMaxConfig m_followerMotorConfig;    
    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_encoder;
    private ElevatorFeedforward m_feedforward;
    private final String m_name;

    private Limits m_limits;

    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;

    private int staleCounter = 0;
    private double lastPosition = 0;

    private double m_setpoint;
    private boolean manualControl;


     /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;


    public ElevatorSubsystem( String _name) {

        m_leader = new SparkMax(ElevatorConstants.kLeaderDeviceId, MotorType.kBrushless);
        m_follower = new SparkMax(ElevatorConstants.kFollowerDeviceId, MotorType.kBrushless);
        m_leaderMotorConfig = new SparkMaxConfig();
        m_followerMotorConfig = new SparkMaxConfig();

        m_leaderMotorConfig.idleMode(IdleMode.kBrake);

        m_leaderMotorConfig.closedLoop
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(ElevatorConstants.kPidValues.p)
        .i(ElevatorConstants.kPidValues.i)
        .d(ElevatorConstants.kPidValues.d)
        .iZone(0)
        .velocityFF(0)
        .outputRange(-1, 1);

        m_followerMotorConfig.idleMode(IdleMode.kBrake);
        m_followerMotorConfig.follow(m_leader);

        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
        SparkMaxUtils.ApplySparkMaxConfig(m_follower, m_followerMotorConfig);


        m_closedLoopController = m_leader.getClosedLoopController();

        m_limits = ElevatorConstants.kLimits;
        m_encoder = m_leader.getEncoder();
        m_name = _name;
        manualControl = true;

        m_feedforward = new ElevatorFeedforward(ElevatorConstants.kFFValues.ks, ElevatorConstants.kFFValues.kg, ElevatorConstants.kFFValues.kv, ElevatorConstants.kFFValues.ka);

         m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                            m_leader.set(-volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Elevator"))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                        getElevatorPosition(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    getRotationsPerSecond(),
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
        m_encoder.setPosition(0);
    }

    public double getElevatorPosition(){
        return m_encoder.getPosition();
    }

    public double getRotationsPerSecond(){
        return m_encoder.getVelocity() / 60;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
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

        m_closedLoopController.setReference(m_prior_iteration_setpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff,
                ArbFFUnits.kVoltage);

    }

   

}