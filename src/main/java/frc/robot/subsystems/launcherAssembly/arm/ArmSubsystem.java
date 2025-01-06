package frc.robot.subsystems.launcherAssembly.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;
public class ArmSubsystem extends SubsystemBase {

    private final TalonFX m_follower;
    private final TalonFX m_leader;

    private boolean emergencyStop;

    private int staleCounter = 0;
    private double lastPosition = 0;
    private double falconErrorCounter = 0;
    private boolean manualControl = true;

    private ArmFeedforward m_feedforward;
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ArmConstants.kPidValues.p,
            ArmConstants.kPidValues.i, ArmConstants.kPidValues.d,
            new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));

    private DutyCycleEncoder m_encoder;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Radians.mutable(0));
    private final MutAngularVelocity m_velocity = (RadiansPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;
    private double falconAngleOffset;
    private double m_globalSetpoint;


    private double usedP;
    private double usedI;
    private double usedD;
    private double usedV;
    private double usedG;

    private DigitalInput m_noteSensor;

    public ArmSubsystem() {

        m_leader = new TalonFX(ArmConstants.kLeaderDeviceId, ArmConstants.kCanName);
        m_follower = new TalonFX(ArmConstants.kFollowerDeviceId, ArmConstants.kCanName);

        m_noteSensor = new DigitalInput(ArmConstants.kNoteSensorDIO);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        m_encoder = new DutyCycleEncoder(0);

        m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, ArmConstants.kFFValues.kg,
                ArmConstants.kFFValues.kv, ArmConstants.kFFValues.ka);
        m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
        // set m_strictFollower to strict-follow m_leader
        // strict followers ignore the leader's invert and use their own

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leader.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the Leader Arm Motor. Restart robot!");
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_follower.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the Leader Arm Motor. Restart robot!");
        }

        m_leader.setNeutralMode(NeutralModeValue.Brake);
        m_follower.setNeutralMode(NeutralModeValue.Brake);

        falconAngleOffset = Math.toRadians(ArmConstants.falconOffsetAngleDegrees);

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
                            log.motor(("Arm"))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                            getFalconAngleRadians(),
                                            Radians))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    getFalconAngularVelocityRadiansPerSec(),
                                                    RadiansPerSecond));

                        },
                        this));

        InitMotionProfile(getArmPosition());


        SmartDashboard.putNumber("ARM p", ArmConstants.kPidValues.p);
        SmartDashboard.putNumber("ARM I", ArmConstants.kPidValues.i);
        SmartDashboard.putNumber("ARM D", ArmConstants.kPidValues.d);
        SmartDashboard.putNumber("ARM V", ArmConstants.kFFValues.kv);
        SmartDashboard.putNumber("ARM G", ArmConstants.kFFValues.kg);

        usedP = ArmConstants.kPidValues.p;
        usedI = ArmConstants.kPidValues.i;
        usedD = ArmConstants.kPidValues.d;
        usedV = ArmConstants.kFFValues.kv;
        usedG = ArmConstants.kFFValues.kg;

    }

    public void resetAfterDisable(){
        System.out.println("Reset After Disable!!!");
        manualControl = true;
        setSpeed(0);
    }

    @Override
    public void periodic() {
        // leave blank
        SmartDashboard.putNumber("Falcon Angle Deg", Math.toDegrees((getFalconAngleRadians())));
        SmartDashboard.putNumber("Falcon Angular Velocity", getFalconAngularVelocityRadiansPerSec());
        SmartDashboard.putNumber("Arm Angle Deg", Math.toDegrees(getArmAngleRadians()));
        SmartDashboard.putNumber("Falcon Angle Error Deg", Math.toDegrees(getFalconAngleRadians() - getArmAngleRadians()));
        SmartDashboard.putNumber("Falcon Voltage", m_leader.get());


        var newP = SmartDashboard.getNumber("ARM p", usedP);
        var newI = SmartDashboard.getNumber("ARM I", usedI);
        var newD = SmartDashboard.getNumber("ARM D", usedD);
        var newV = SmartDashboard.getNumber("ARM V", usedV);
        var newG = SmartDashboard.getNumber("ARM G", usedG);
        
        if (newP != usedP && newP != 0) {
            usedP = newP;
            m_controller.setP(usedP);
            System.out.println("Using P of " + usedP);
        }

        if (newI != usedI && newI != 0) {
            usedI = newI;
            m_controller.setI(usedI);
            System.out.println("Using I of " + usedI);

        }

        if (newD != usedD && newD != 0) {
            usedD = newD;
            m_controller.setD(usedD);
            System.out.println("Using D of " + usedD);

        }

        if (newV != usedV && newV != 0) {
            usedV = newV;
            System.out.println("Using V of " + usedV);
            m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, usedG, usedV, ArmConstants.kFFValues.ka);
        }

        if (newG != usedG && newG != 0) {
            usedG = newG;
            System.out.println("Using G of " + usedG);
            m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, usedG, usedV, ArmConstants.kFFValues.ka);
        }




        if (Math.abs(getFalconAngularVelocityRadiansPerSec()) < .1) {
            double falconError = Math.abs(getFalconAngleRadians() - getArmAngleRadians());
            if (falconError > ArmConstants.falconErrorThresh) {
                falconErrorCounter++;
            } else {
                falconErrorCounter = 0;
            }
            if (falconErrorCounter > ArmConstants.falconErrorCount) {
                falconAngleOffset += (getArmAngleRadians() - getFalconAngleRadians());
                falconErrorCounter = 0;
            }

        }else{
            falconErrorCounter = 0;
        }
        if(!manualControl){

            RunArmToPos();
        }
    }

    public boolean hasNote(){
        return !m_noteSensor.get();
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_leader.set(speed);
    }

    public double getArmPosition() {
        return getFalconAngleRadians();
    }

    public double getFalconAngleRadians() {
        return (m_leader.getPosition().refresh().getValueAsDouble() / 31.8181818) * 2 * Math.PI + falconAngleOffset;
    }

    public double getFalconAngularVelocityRadiansPerSec() {
        return (m_leader.getVelocity().refresh().getValueAsDouble() / 31.8181818) * 2 * Math.PI;
    }

    public double getArmAngleRadians() {
        return m_encoder.get() * 2 * Math.PI
                + Math.toRadians(ArmConstants.armEncoderOffsetAngleDegrees);
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        m_controller.reset(getArmPosition());
        m_controller.setTolerance(ArmConstants.kPositionTolerance);
        m_controller.setGoal(new TrapezoidProfile.State(setpoint, 0));
        m_globalSetpoint = setpoint;
    }

    public void RunArmToPos() {

        double pid_output = m_controller.calculate(getArmPosition(), m_globalSetpoint);
        double ff = m_feedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);
        SmartDashboard.putNumber("Arm Pid Output", pid_output);
        SmartDashboard.putNumber("Arm FF Output", ff);
        SmartDashboard.putNumber("Arm Position Eror", Math.toDegrees(m_controller.getPositionError()));

        m_leader.setVoltage(ff  + pid_output);
        SmartDashboard.putNumber("SetpointVelocity", m_controller.getSetpoint().velocity);
        SmartDashboard.putNumber("SetpointPosition", Math.toDegrees(m_controller.getSetpoint().position));
         SmartDashboard.putNumber("Arm Set Voltage", ff + pid_output);
        SmartDashboard.putNumber("Global Setpoint ", Math.toDegrees(m_globalSetpoint));
        
        //if we are about to overextend, instantly stop and go back to intake position.
        if (getArmAngleRadians() < ArmConstants.kLimits.high) {
            StopArm();
            emergencyStop = true;
            InitMotionProfile(ArmConstants.intakeAngle);
        }
    }

    /**
     * Stop the motor
     */
    public void StopArm() {
        setSpeed(0);
    }

    /**
     * @brief Checks if the arm has reached its target
     * @return true if the arm has reached its target, false otherwise
     */
    private Boolean armReachedTarget() {

        // check if the arm has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(getArmPosition() - lastPosition) < ArmConstants.kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = getArmPosition();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(getArmPosition() - m_controller.getGoal().position);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < ArmConstants.kDiffThreshold
                || (delta < ArmConstants.kStaleTolerance && staleCounter > ArmConstants.kStaleThreshold);
    }

    private Boolean isFinished() {
        System.out.println("emergency stop " + emergencyStop + ", " + "arm target: " + armReachedTarget());

         var isFinished = emergencyStop || armReachedTarget();
        return isFinished;// isFinished;
    }

    private Boolean isWithinLimits(double direction) {
        /*
         * Below needs to be verified with setup
         * return (m_leader.get() < 0 && getArmPosition() > ArmConstants.kLimits.low)
         * || (m_leader.get() > 0 && getArmPosition() < ArmConstants.kLimits.high);
         */
        return getArmAngleRadians() > ArmConstants.kLimits.high || ( direction > 0);
    }


    private double sanitizePositionSetpoint(double setpoint){
        if (setpoint > ArmConstants.kLimits.low){
            setpoint = ArmConstants.kLimits.low;
        }
        if(setpoint < ArmConstants.kLimits.high){
            setpoint = ArmConstants.kLimits.high;
        }
        return setpoint;
    }

    /**
     * 
     * @param setpoint the desired arm position IN RADIANS
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be
     *       called when the command finishes.
     * @return
     */

    public Command RunArmToPositionCommand(double setpoint) {
        return new FunctionalCommand(
                () -> {
                    double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
                    System.out.println("-----------------Starting Arm to position " + sanitizedSetpoint + " --------------");
                    InitMotionProfile(sanitizedSetpoint);
                    manualControl = false;
                },
                () -> {},
                (interrupted) -> {
                    emergencyStop = false;
                },
                () -> {
                    return isFinished();
                }, this);
    }


    public void CalculateNewAutoAngle(DriveSubsystem robotDrive){
        double new_setpoint = MathUtils.angleRadiansToScoringTarget(robotDrive.getPose());
        double sanitizedSetpoint = sanitizePositionSetpoint(new_setpoint);
        if(sanitizedSetpoint != m_globalSetpoint){
            System.out.println("-----------------New Arm to Setpoint " + sanitizedSetpoint + " --------------");
            System.out.println("Current ARm Angle: " + Math.toDegrees(getArmPosition()));
        }
        m_globalSetpoint = sanitizedSetpoint;
    }

    public void UpdateAngleManually(double diff){
        m_globalSetpoint += diff;
    }


    public Command RunArmToAutoPositionCommand(DriveSubsystem robotDrive, boolean stopOnFinish) {
        return new FunctionalCommand(
                () -> {
                    double setpoint = MathUtils.angleRadiansToScoringTarget(robotDrive.getPose());
                    double sanitizedSetpoint = sanitizePositionSetpoint(setpoint);
                    System.out.println("-----------------Starting Arm to position " + setpoint + ", Sanitized = " + sanitizedSetpoint + " --------------");
                    InitMotionProfile(sanitizedSetpoint);
                    manualControl = false;
                },
                () -> {
                    CalculateNewAutoAngle(robotDrive);
                },
                (interrupted) -> {
                    emergencyStop = false;
                },
                () -> {
                    return stopOnFinish && isFinished();
                }, this);
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Up Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return !isWithinLimits(getSpeed.getAsDouble());
                }, this);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Down Starting--------------");
                    manualControl = true;
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return !isWithinLimits(getSpeed.getAsDouble());
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}