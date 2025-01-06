package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public class ClimbMotorSubsystem extends SubsystemBase{
    private SparkMax m_motor;
    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_encoder;
    private SimpleMotorFeedforward m_feedforward;
    private SparkMaxConfig m_motorConfig;
    private final String m_name;

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_distance = Rotations.mutable(0);
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);
    private final SysIdRoutine m_sysIdRoutine;

    public ClimbMotorSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String name,boolean isInverted) {

        m_name = name;

        m_motor = new SparkMax(deviceId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_closedLoopController = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();
        
        

        m_motorConfig.idleMode(IdleMode.kBrake);

        m_motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(pidValues.p)
        .i(pidValues.i)
        .d(pidValues.d)
        .iZone(0)
        .velocityFF(0)
        .outputRange(-1, 1);

        
        SparkMaxUtils.ApplySparkMaxConfig(m_motor, m_motorConfig);


        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                            m_motor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Climb-" + m_name))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(m_encoder.getPosition(), Rotations))
                                    // CRITICAL - encoder returns RPM, we need RPS here. Hence / 60
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_encoder.getVelocity() / 60, RotationsPerSecond));

                        },
                        this));


    }
    
    

    @Override
    public void periodic() {

    }

    /**
     * 
     * Set the speed of the *climb* motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunClimb(int setpoint) {

        double ff = m_feedforward.calculate(setpoint / 60);  //important, calculate needs rps, not rpm. Hence, / 60
        m_closedLoopController.setReference(setpoint, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);

        

    }

    public void StopClimb() {
        setSpeed(0);
    }


    public Command RunClimbForwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Climb forward--------------");
                },
                () -> setSpeed(1),
                (interrupted) -> StopClimb(),
                () -> false, this);
    }

    public Command RunClimbBackwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Climb Reverse--------------");
                },
                () -> setSpeed(-1),
                (interrupted) -> StopClimb(),
                () -> false, this);
    }



    /***********************************************************
     * SYSID functions
     ***********************************************************/
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}

