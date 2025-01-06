package frc.robot.subsystems.launcherAssembly.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;
public class TalonShooterWheelSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final Boolean m_invert;
    private final String name;
    private double m_goalSpeed;
    private boolean m_shooterStopped= true;

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0);
    private SimpleMotorFeedforward m_feedforward;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;



    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;


   

    public TalonShooterWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name,
            boolean isInverted) {

        m_motor = new TalonFX(deviceId, "rio");
        TalonFXConfiguration configs = new TalonFXConfiguration();
        m_invert = isInverted;
        name = _name;
        
        configs.Slot0.kP = pidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = pidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = pidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.0;// 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33
                               // = 0.12
                               // volts / Rotation per second
        configs.Slot0.kA = 0.0;
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.CurrentLimits.StatorCurrentLimit = 50;
        configs.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);

        if (!TalonUtils.ApplyTalonConfig(m_motor, configs)) {
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
                                    .angularPosition(m_distance.mut_replace(m_motor.getPosition().refresh().getValue(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_motor.getVelocity().refresh().getValue(),
                                                    RotationsPerSecond));

                        },
                        this));
        InitMotionProfile(8000,8000);
    }

    @Override
    public void periodic() {

        RunShooterWithMotionProfile();
    }

    public void setTargetSpeed(double speed) {
        m_goalSpeed = speed;
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
        return TalonUtils.getSpeedRotationsPerMinute(m_motor);
    }

    public boolean isAtDesiredSpeed() {
        SmartDashboard.putNumber("Shooter " + name + "Desired Speed Offset", Math.abs(getSpeedRotationsPerMinute()
                - m_goalSpeed));

        SmartDashboard.putNumber("Shooter " + name + "Running Speed ", getSpeedRotationsPerMinute());

        return Math.abs(getSpeedRotationsPerMinute()
                - m_goalSpeed) < Constants.ShooterConstants.kTolerance;
    }

    public boolean isShooterRunningFaster(){
        return getSpeedRotationsPerMinute() - m_goalSpeed > 0 && getSpeedRotationsPerMinute() - m_goalSpeed < 500;
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunShooterWithMotionProfile() {
        if (!m_shooterStopped) {
            TrapezoidProfile.State setpoint = m_profile.calculate(0.02, m_prior_iteration_setpoint,new TrapezoidProfile.State(m_goalSpeed, 0));
            //System.out.println("Running Shooter. Setpoint = " + setpoint.position);
            double ff = m_feedforward.calculate(setpoint.position / 60); // important, calculate
            // needs rps, not rpm. Hence, / 60
            m_motor.setControl(m_voltageVelocity.withFeedForward(ff).withVelocity(setpoint.position / 60));
            m_prior_iteration_setpoint = setpoint;
        }
        SmartDashboard.putNumber(name + " Shooter Fwd vel", getSpeedRotationsPerMinute());
    }

    /**
     * Stop the motor
     */
    public void StopShooter() {
        System.out.println("-----------------Stopping shooter--------------");
        m_shooterStopped = true;
        setSpeed(0);
        
    }


    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double maxV, double maxA) {
        System.out.println("Shooter Motion Profile setpoint = " + m_goalSpeed + ", Current = " + getSpeedRotationsPerMinute());
        m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxV, maxA));
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getSpeedRotationsPerMinute(), 0);
    }


    public void StartShooter(){
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getSpeedRotationsPerMinute(), 0);
        m_shooterStopped = false;
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
                    StartShooter();
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

    public Command RunShooterCommandUntilAtSpeed() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter until at speed--------------");
                    StartShooter();
                },
                () -> {},
                (interrupted) -> {
                   
                    if (interrupted){
                        StopShooter();
                    }
                },
                () -> {
                    return (isAtDesiredSpeed()) || m_shooterStopped;
                }, this);

    }

    public Command RunShooterUntilStopped(){
        return new StartEndCommand(
                () -> {
                    System.out.println("-----------------Starting shooter infinite--------------");
                    StartShooter();
                },
                () -> {
                    StopShooter();
                }, this);
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}
