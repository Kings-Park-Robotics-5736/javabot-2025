package frc.robot.subsystems.launcherAssembly.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.field.ScoringPositions;
import frc.robot.utils.MathUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;



enum ShooterState{
    STOPPED,
    RUNNING,
    IDLING,
    AMP,
    SCORPION
}
public class ShooterWheelSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final Boolean m_invert;
    private final String name;
    private double m_forwardSpeed;
    private double m_reverseSpeed;
    private double startTime = 0;

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0);
    private SimpleMotorFeedforward m_feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;

    private ShooterState m_shooterState;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;
    private double desired_speed;



    public ShooterWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name,
            boolean isInverted, double forwardSpeed, double reverseSpeed) {

        m_motor = new TalonFX(deviceId, "rio");
        TalonFXConfiguration configs = new TalonFXConfiguration();
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        m_invert = isInverted;
        name = _name;
        m_forwardSpeed = forwardSpeed;
        m_reverseSpeed = reverseSpeed;

        configs.Slot0.kP = pidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = pidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.0;// 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33
                               // = 0.12
                               // volts / Rotation per second
        configs.Slot0.kA = 0.0;// 3.00;
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.CurrentLimits.StatorCurrentLimit = 50;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_shooterState= ShooterState.STOPPED;

        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);

        for (int i = 0; i < 5; ++i) {
            status = m_motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
            else {
                System.out.println("Motor Initialization Failed");
            }

        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the " + name + " Shooter Motor. Restart robot!");
        }
        m_motor.setNeutralMode(NeutralModeValue.Coast);
        m_motor.setInverted(isInverted);

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
                            log.motor(("shooter-" + name))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(m_motor.getPosition().refresh().getValueAsDouble(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_motor.getVelocity().refresh().getValueAsDouble(),
                                                    RotationsPerSecond));

                        },
                        this));
    }

    @Override
    public void periodic() {

        if(m_shooterState != ShooterState.STOPPED){
            RunShooterWithMotionProfile();
        }

        
    }

    public void setNewForwardSpeed(double speed) {
        m_forwardSpeed = speed;
        if(m_shooterState == ShooterState.RUNNING){
            InitMotionProfile(m_forwardSpeed);
            desired_speed = m_forwardSpeed;
        }else if (m_shooterState == ShooterState.IDLING && speed < 4000){
            InitMotionProfile(speed/2);
            desired_speed = speed/2;
        }
    }

    public void setNewReverseSpeed(double speed) {
        m_reverseSpeed = speed;
    }
    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        
        m_motor.set(speed);
    }

    public int getSpeedRotationsPerMinute() {
        double rpm = m_motor.getVelocity().refresh().getValueAsDouble() * 60;
        return (int) rpm;
    }

    public boolean isAtDesiredSpeed() {
        SmartDashboard.putNumber("Shooter " + name + "Desired Speed Offset", Math.abs(getSpeedRotationsPerMinute()
                - desired_speed));

        SmartDashboard.putNumber("Shooter " + name + "Running Speed ", getSpeedRotationsPerMinute());
        return Math.abs(getSpeedRotationsPerMinute()
                - desired_speed) < Constants.ShooterConstants.kTolerance;
    }

    public boolean isShooterRunningFaster(){
        return getSpeedRotationsPerMinute() - desired_speed > 0 && getSpeedRotationsPerMinute() - desired_speed < 500;
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunShooterWithMotionProfile() {
        TrapezoidProfile.State setpoint = profile.calculate(0.02, m_prior_iteration_setpoint,new TrapezoidProfile.State(desired_speed, 0));
        //System.out.println("Running Shooter. Setpoint = " + setpoint.position);
        double ff = m_feedforward.calculate(setpoint.position / 60); // important, calculate
        // needs rps, not rpm. Hence, / 60
        m_motor.setControl(m_voltageVelocity.withFeedForward(ff).withVelocity(setpoint.position / 60));
        m_prior_iteration_setpoint = setpoint;
    }

    /**
     * Stop the motor
     */
    public void StopShooter() {
        System.out.println("-----------------Stopping shooter--------------");
        setSpeed(0);
        m_shooterState = ShooterState.STOPPED;
    }


    
    private void InitMotionProfile(double setpoint) {
        InitMotionProfile(setpoint, 8000, 8000);
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint, double maxV, double maxA) {
        System.out.println("Shooter Motion Profile setpoint = " + setpoint + ", Current = " + getSpeedRotationsPerMinute());
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxV, maxA));
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getSpeedRotationsPerMinute(), 0);

        startTime = Timer.getFPGATimestamp();

    }


    public void SpoolShooter(){
        desired_speed = m_forwardSpeed;
        InitMotionProfile(m_forwardSpeed);
        m_shooterState = ShooterState.RUNNING;
    }

    public boolean shooterAtSpeed(){
        return isAtDesiredSpeed();
    }

    /**
     * 
     * @param FinishWhenAtTargetSpeed When this is set, this command should finish
     *                                once a call to isAtDesiredSpeed returns true.
     *                                When this is false, this command should never
     *                                end.
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be
     *       called when the command finishes.
     * @return
     */


    public Command EjectCommand(){
        return new FunctionalCommand(() -> {
                    System.out.println("-----------------Starting shooter EJECT--------------");
                 
                    m_shooterState = ShooterState.STOPPED;
                },
                () -> {
                    setSpeed(.35);
                },
                (interrupted) -> {                   
                        StopShooter();
                },
                () -> {
                    return false;
                }, this);
    }
    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward--------------");
                    InitMotionProfile(m_forwardSpeed);
                    m_shooterState = ShooterState.RUNNING;
                    desired_speed = m_forwardSpeed;
                },
                () -> {
                    SmartDashboard.putNumber(name + " Shooter Fwd vel", getSpeedRotationsPerMinute());
                },
                (interrupted) -> {
                    if (!FinishWhenAtTargetSpeed) {
                        StopShooter();
                    }

                    if (interrupted){
                        StopShooter();
                    }
                },
                () -> {
                    return (FinishWhenAtTargetSpeed && isAtDesiredSpeed()) || m_shooterState ==ShooterState.STOPPED;
                }, this);

    }

      public Command RunShooterForwardForAmp() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward AMP--------------");
                    InitMotionProfile(ShooterConstants.kAmpSpeed);
                    desired_speed = ShooterConstants.kAmpSpeed;
                    m_shooterState = ShooterState.AMP;
                },
                () -> {
                    SmartDashboard.putNumber(name + " Shooter Fwd vel", getSpeedRotationsPerMinute());
                },
                (interrupted) -> {
                    StopShooter();
                },
                () -> {
                    return false;
                }, this);

    }

    public Command RunShooterForwardForScorpion(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward SCORPION--------------");
                    InitMotionProfile(ShooterConstants.scorpionSpeed,100000,100000);
                    desired_speed = ShooterConstants.scorpionSpeed;
                    m_shooterState = ShooterState.SCORPION;
                },
                () -> {
                    SmartDashboard.putNumber(name + " Shooter Fwd vel", getSpeedRotationsPerMinute());
                },
                (interrupted) -> {
                    if (!FinishWhenAtTargetSpeed) {
                        StopShooter();
                    }

                    if (interrupted){
                        StopShooter();
                    }
                },
                () -> {
                    return (FinishWhenAtTargetSpeed && isAtDesiredSpeed()) || m_shooterState ==ShooterState.STOPPED;
                }, this);

    }


    public Command RunShooterForwardIdle() {

        return this.runOnce(()->{
            System.out.println("-----------------Starting shooter IDLE Speed = " + ShooterConstants.kIdleSpeed + "--------------");
                    InitMotionProfile(ShooterConstants.kIdleSpeed);
                    desired_speed = ShooterConstants.kIdleSpeed;
                    m_shooterState = ShooterState.IDLING;
        });
    }

    public Command RunShooterBackwardCommand(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter Backward--------------");
                    //InitMotionProfile(m_reverseSpeed, 10000, 10000);
                    //desired_speed = m_reverseSpeed;
                    m_shooterState = ShooterState.STOPPED;
                },
                () -> {
                    setSpeed(-.10);
                },
                (interrupted) -> {
                    System.out.println("Shooter Backward is Stopped! Interrupted = "  + interrupted);
                    if (!FinishWhenAtTargetSpeed) {
                        StopShooter();
                    }
                },
                () -> {
                    return FinishWhenAtTargetSpeed && isAtDesiredSpeed();
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}
