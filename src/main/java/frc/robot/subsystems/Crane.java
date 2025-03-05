
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.CraneState;
import frc.robot.players.PlayerConfigs;
import frc.robot.utils.Logger;

public class Crane extends SubsystemBase {
    // Crane vars
    public CraneState craneState = CraneState.STATION; // stow is the starting configuration
    // private ArmFeedforward feedforward;
    /** Degree of angleMargin so that the crane can progress to the next position. */
    private double angleMargin = 8;
    private double extensionMargin = 0.5;

    // Gripper
    private final SparkMax gripperMotor;
    private final SparkMaxConfig gripperConfig;
    public final SparkClosedLoopController gripperPID;
    public double gripperSetpoint;
    // public boolean gripState = true; // gripToggle = true: means gripping coral, false: means open

    // Wrist
    private final SparkMax wristMotor;
    private final SparkMaxConfig wristConfig;
    public final SparkClosedLoopController wristPID;
    public double wristSetpoint;
    private double prevWristDirection = 0;
    
    // Arm: Elbow and Extender
    
    // Elbow
    private final SparkMax elbowMotor;
    private final SparkMaxConfig elbowConfig;
    public final SparkClosedLoopController elbowPID;
    public double elbowSetpoint;
    private double prevElbowDirection = 0;
    
    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;
    public double extenderSetpoint;

    //Sucker
    private final SparkMax suckerMotor;
    private final SparkMaxConfig suckerConfig;
    public final SparkClosedLoopController suckerPID;
    public double suckerSetpoint;
    private boolean prevHoldState = false; // used to grab position once for holding sucker in place
    
    public Crane() {
        // feedforward = new ArmFeedforward(CraneConstants.kSVolts, 
        //                                  CraneConstants.kGVolts, 
        //                                  CraneConstants.kVVoltSecsPerDeg, 
        //                                  CraneConstants.kAVoltSecsSquaredPerDeg
        // );

        gripperMotor = new SparkMax(CANIDS.GRIPPER, MotorType.kBrushless);
        gripperConfig = new SparkMaxConfig();
        gripperPID = gripperMotor.getClosedLoopController();

        wristMotor = new SparkMax(CANIDS.WRIST, MotorType.kBrushless);
        wristConfig = new SparkMaxConfig();
        wristPID = wristMotor.getClosedLoopController();

        elbowMotor = new SparkMax(CANIDS.ELBOW, MotorType.kBrushless);
        elbowConfig = new SparkMaxConfig();
        elbowPID = elbowMotor.getClosedLoopController();

        extenderMotor = new SparkMax(CANIDS.EXTENDER, MotorType.kBrushless);
        extenderConfig = new SparkMaxConfig();
        extenderPID = extenderMotor.getClosedLoopController();

        suckerMotor = new SparkMax(CANIDS.SUCKER, MotorType.kBrushless);
        suckerConfig = new SparkMaxConfig();
        suckerPID = suckerMotor.getClosedLoopController();

        gripperConfig
            .smartCurrentLimit(20)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        gripperConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kGripperCeiling)
            .reverseSoftLimit(CraneConstants.kGripperHardDeck);
        gripperConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse);
        gripperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.0, 0.0);
            
        gripperMotor.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        wristConfig
            .smartCurrentLimit(20)
            .idleMode(IdleMode.kBrake);
        wristConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kWristCeiling)
            .reverseSoftLimit(CraneConstants.kWristHardDeck - 20);
        wristConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.000003, 0.1);
            
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elbowConfig
            .smartCurrentLimit(80)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        elbowConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(CraneConstants.kElbowCeiling + 2)
            .reverseSoftLimit(CraneConstants.kElbowHardDeck - 2);
        elbowConfig.encoder
            .positionConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse);
        elbowConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing
            .pid(0.005, 0.000003, 0.1);
            // .pid(0.0, 0.0, 0.0);
            
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        // extenderConfig.softLimit
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimitEnabled(true)
        //     .forwardSoftLimit(inchesToDegrees(3))
        //     .reverseSoftLimit(inchesToDegrees(CraneConstants.kExtenderCeiling));
        extenderConfig.encoder
            .positionConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse);
        extenderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing, 1/25/2025 was: 0.01, 0.01, 0.1
            .pid(0.005, 0.0, 0.1);
            
        extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //FLEX Motor TODO: write code for spark flex motor
        suckerConfig
            .inverted(true)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        suckerConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kSuckerEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kSuckerEncoderDistancePerPulse);
        suckerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.01, 0.0, 0.0);
            
        suckerMotor.configure(suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set up setpoints for each motor
        // setGripperPosition(CraneConstants.kGripperHardDeck);
        setWristPosition(CraneConstants.kWristHardDeck);
        setElbowPosition(CraneConstants.kElbowHardDeck);
        setExtenderPosition(CraneConstants.kExtenderStart);
    }

    public void goToCraneState(CraneState state) {
        if(state == CraneState.CLIMB) {
            putCraneToClimbState();
        }
        if(state == CraneState.STATION) {
            setWristPosition(CraneConstants.kWristStation);
            // Move up with extender at start so that we don't retract into the frame if below station
            if(upToElbowPosition(CraneConstants.kElbowStation - 8, CraneConstants.kExtenderStart)) {
                if(downToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)
                    && upToElbowPosition(CraneConstants.kElbowStation, CraneConstants.kExtenderLimit1)) {
                    setWristPosition(CraneConstants.kWristStation);
                    setElbowPosition(CraneConstants.kElbowStation);
                    setExtenderPosition(CraneConstants.kExtenderStation);
                }
            }
        }
        if(state == CraneState.SHELF) {
            setWristPosition(CraneConstants.kWristShelf);
            if(downToElbowPosition(-CraneConstants.kElbowHorizonOffset - 10, CraneConstants.kExtenderLimit1)) {
                if(downToElbowPosition(CraneConstants.kElbowShelf, CraneConstants.kExtenderShelf)
                   && upToElbowPosition(CraneConstants.kElbowShelf, CraneConstants.kExtenderShelf)) {
                    setWristPosition(CraneConstants.kWristShelf);
                    setElbowPosition(CraneConstants.kElbowShelf);
                    setExtenderPosition(CraneConstants.kExtenderShelf);
                }
            }
        }
        if(state == CraneState.LOW_REEF) {
            setWristPosition(CraneConstants.kWristLow);
            if(upToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)
                && downToElbowPosition(CraneConstants.kElbowLow, CraneConstants.kExtenderLimit1)) {
                setWristPosition(CraneConstants.kWristLow);
                setElbowPosition(CraneConstants.kElbowLow);
                setExtenderPosition(CraneConstants.kExtenderLow);
            }
        }
        if(state == CraneState.MID_REEF) {
            setWristPosition(CraneConstants.kWristMid);
            if(upToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)
                && downToElbowPosition(CraneConstants.kElbowMid, CraneConstants.kExtenderLimit1)) {
                setWristPosition(CraneConstants.kWristMid);
                setElbowPosition(CraneConstants.kElbowMid);
                setExtenderPosition(CraneConstants.kExtenderMid);
            }
        }
        if(state == CraneState.HIGH_REEF) {
            setWristPosition(CraneConstants.kWristHardDeck);
            if(upToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)
                && downToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)) {
                setWristPosition(CraneConstants.kWristHigh);
                setElbowPosition(CraneConstants.kElbowHigh);
                setExtenderPosition(CraneConstants.kExtenderHigh);
            }
        }
        // if(state == CraneState.STOW) {
        //     // If we want to go to elbow pause, we must retract extender
        //     setWristPosition(CraneConstants.kWristHardDeck);
        //     if(downToElbowPosition(CraneConstants.kElbowHardDeck, CraneConstants.kExtenderLimit1)) {
        //         // If elbow is at hard deck, then we slightly retract the extender for stability
        //         setWristPosition(CraneConstants.kWristHardDeck);
        //         setElbowPosition(CraneConstants.kElbowHardDeck);
        //         setExtenderPosition(CraneConstants.kExtenderStart);
        //     }
        // }
    }

    /**
     * Puts the crane to climb state and sets the craneState to CLIMB to keep hold.
     */
    public void putCraneToClimbState() {
        // This sets the crane to the climb state only if this method is called
        craneState = CraneState.CLIMB; // climb state
        setWristPosition(CraneConstants.kWristHardDeck);
        if(upToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)
           && downToElbowPosition(CraneConstants.kElbowHigh, CraneConstants.kExtenderLimit1)) {
            setWristPosition(CraneConstants.kWristHigh-30);
            setElbowPosition(CraneConstants.kElbowHigh);
            setExtenderPosition(CraneConstants.kExtenderHardDeck);
        }
        
        SmartDashboard.putString("Crane State", craneState.toString());
        SmartDashboard.putBoolean("Crane FC", PlayerConfigs.fineControlCraneEnable);
        SmartDashboard.putNumber("FC Elbow", PlayerConfigs.fineControlElbow);
        SmartDashboard.putNumber("FC Wrist", PlayerConfigs.fineControlWrist);
        SmartDashboard.putNumber("FC Extender", PlayerConfigs.fineControlExtender);
    }

    /**
     * Rotate elbow upwards (CEILING direction) with extender retracted to a limit.
     * 
     * @param elbowPosition The target elbow position in degrees.
     * @param extenderLimit The target extender position in inches.
     * @return True if the elbow is at the desired position.
     */
    private boolean upToElbowPosition(double elbowPosition, double extenderLimit) {
        // If we want to go to elbow position, we must retract extender, then we rotate elbow
        if(getElbowPosition() + angleMargin < elbowPosition) {
            if(getExtenderPosition() - extensionMargin > extenderLimit
               || getExtenderPosition() + extensionMargin < extenderLimit) {
                setExtenderPosition(extenderLimit);
            } else {
                setElbowPosition(elbowPosition);
            }
            return false;
        }
        return true;
    }

    /**
     * Rotate elbow upwards (HARD DECK direction) with extender retracted to a limit.
     * 
     * @param elbowPosition The target elbow position in degrees.
     * @param extenderLimit The target extender position in inches.
     * @return True if the elbow is at the desired position.
     */
    private boolean downToElbowPosition(double elbowPosition, double extenderLimit) {
        // If we want to go to elbow position, we must retract extender, then we rotate elbow
        if(getElbowPosition() - angleMargin > elbowPosition) {
            if(getExtenderPosition() - extensionMargin > extenderLimit
               || getExtenderPosition() + extensionMargin < extenderLimit) {
                setExtenderPosition(extenderLimit);
            } else {
                setElbowPosition(elbowPosition);
            }
            return false;
        }
        return true;
    }

    /**
     * Sets the angle of the gripper, shaft CW+.
     * 
     * @param setPoint The desired angle of the gripper in degrees
     */
    // public void setGripperPosition(double setPoint) {
    //     gripperSetpoint = filterSetPoint(setPoint,
    //                                      CraneConstants.kGripperHardDeck,
    //                                      CraneConstants.kGripperCeiling);
    //     gripperPID.setReference(gripperSetpoint, ControlType.kPosition);
    //     SmartDashboard.putNumber("Gripper Setpoint", setPoint);
    // }
    
    /**
     * This method spins the sucker motor based on voltage and the direction 
     * provided by a sign (e.g. -12).
     * 
     * @param volts The volts to spin the sucker motor, max around (+/-) 12 volts.
     */
    public void spinSucker(double volts) {
        // suckerPID.setReference(velocity, ControlType.kVelocity);
        suckerMotor.setVoltage(volts);
    }

    /**
     * Holds the sucker in place if it is not being ejected or intaked. Grabs position once and holds it.
     */
    public void holdSucker() {
        if (!PlayerConfigs.suckerEject && !PlayerConfigs.suckerIntake && !prevHoldState) {
            spinSucker(0);
            suckerSetpoint = getSuckerPosition();
            // System.out.println("SuckerSetpoint" + suckerSetpoint);
        }
        // prevHoldState = !PlayerConfigs.suckerEject && !PlayerConfigs.suckerIntake;
        // suckerPID.setReference(suckerSetpoint, ControlType.kPosition);
        spinSucker(0);
    }

    /**
     * Sets the angle of the wrist, shaft CCW+.
     * 
     * @param setPoint The desired angle of the wrist in degrees
     */
    public void setWristPosition(double setPoint) {
        if(craneState == CraneState.HIGH_REEF) {
            wristSetpoint = filterSetPoint(setPoint, 
                                           CraneConstants.kWristHigh - 15, 
                                           CraneConstants.kWristHardDeck);
        } 
        else if(craneState == CraneState.CLIMB) {
            wristSetpoint = filterSetPoint(setPoint, 
                                           CraneConstants.kWristHigh, 
                                           CraneConstants.kWristHardDeck);
        }
        else {
            wristSetpoint = filterSetPoint(setPoint, 
                                        CraneConstants.kWristHardDeck, 
                                        CraneConstants.kWristCeiling);
        }
        /*
         * don't let integral accumulate if more than a few degrees away from setpoint
         */
        if(Math.abs(getRelativeWristPos() - wristSetpoint) > 10 ||
           Math.abs(getRelativeWristPos() - wristSetpoint) < 0.25 ||
           prevWristDirection != Math.signum(getRelativeWristPos() - wristSetpoint)) {
            wristPID.setIAccum(0.0);
        }
        prevWristDirection = Math.signum(getRelativeWristPos() - wristSetpoint);
        setPoint = getElbowPosition() + wristSetpoint;
        wristPID.setReference(setPoint, ControlType.kPosition);
        SmartDashboard.putNumber("Wrist SetP", wristSetpoint);
        SmartDashboard.putNumber("Wrist Pos", getRelativeWristPos());
    }

    /**
     * Sets the angle of the elbow, shaft CW+.
     * 
     * @param setPoint The desired angle of the elbow in degrees
     */
    public void setElbowPosition(double setPoint) {
        elbowSetpoint = filterSetPoint(setPoint, 
                                       CraneConstants.kElbowHardDeck, 
                                       CraneConstants.kElbowCeiling);
        /*
         * don't let integral accumulate if more than a few degrees away from setpoint
         */
        if(Math.abs(getElbowPosition() - elbowSetpoint) > 10 ||
           Math.abs(getElbowPosition() - elbowSetpoint) < 0.25 ||
           prevElbowDirection != Math.signum(getElbowPosition() - elbowSetpoint)) {
            elbowPID.setIAccum(0.0);
        }
        prevElbowDirection = Math.signum(getElbowPosition() - elbowSetpoint);
        /*
         * Set this value to the voltage reading when crane is retracted and elbow is at horizon, like in mid / low reef
         */
        double voltsAtMaxHorizon = 1.0; // 1.31 V calculated, try lower and build up
        double voltsAtMinHorizon = 0.43; // 0.56 V calculated, try lower and build up
        double voltsInUse = 0.0;

        // Angle of elbow from horizon line
        double angleFromHorizon = getElbowPosition() + CraneConstants.kElbowHorizonOffset;

        // Scale the voltage to the extender position as a percentage of the ceiling
        voltsInUse = (getExtenderPosition() / CraneConstants.kExtenderCeiling) * (voltsAtMaxHorizon - voltsAtMinHorizon) + voltsAtMinHorizon;

        double counterGravityVolts = voltsInUse * Math.cos(Math.toRadians(angleFromHorizon));

        // double appliedVoltageElbow = elbowMotor.getBusVoltage() * elbowMotor.getAppliedOutput();
        // System.out.format("Controller output: %8.5f V\tGravity counter: %8.5f V\tElbow from horizon: %8.5f deg\tExtender: %8.5f\n", 
        //                    appliedVoltageElbow, counterGravityVolts, angleFromHorizon, extenderSetpoint); // confirm with REV Hardware Client, elbow canID 11

        elbowPID.setReference(elbowSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, counterGravityVolts);
        SmartDashboard.putNumber("Elbow SetP", elbowSetpoint);
        SmartDashboard.putNumber("Elbow Pos", getElbowPosition());
    }

    /**
     * Sets the extension of the extender, setpoint and actual position are flipped.
     * Full extension is setpoint = ceiling, motor = 0.
     * Full retraction is setpoint = 0, motor = ceiling.
     * 
     * @param setPoint The desired extension of the extender in inches
     */
    public void setExtenderPosition(double setPoint) {
        // Full extension is setpoint = ceiling, motor = 0
        // Full retraction is setpoint = 0, motor = ceiling
        extenderSetpoint = filterSetPoint(setPoint, 
                                          CraneConstants.kExtenderHardDeck, 
                                          CraneConstants.kExtenderCeiling);
        setPoint = CraneConstants.kExtenderStart - extenderSetpoint;
        setPoint = inchesToDegrees(setPoint);
        extenderPID.setReference(setPoint, ControlType.kPosition);
        // System.out.println("Extender input: " + setPoint);
        SmartDashboard.putNumber("Extender SetP", extenderSetpoint);
        SmartDashboard.putNumber("Extender Pos", getExtenderPosition());
    }

    private double inchesToDegrees(double inches) {
        return inches * 360 / CraneConstants.kPullyCircumferenceInches;
    }

    private double degreesToInches(double degrees) {
        return degrees * CraneConstants.kPullyCircumferenceInches / 360;
    }
    
    /**
     * Returns the ceiling if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param setPoint The desired state of the crane.
     * @param hardDeck The hard deck of the crane.
     */
    private double filterSetPoint(double setPoint, double hardDeck, double ceiling) {
        if(setPoint < hardDeck)
            setPoint = hardDeck;
        if(setPoint > ceiling)
            setPoint = ceiling;
        return setPoint;
    }

    /** Returns the current angle of the elbow in degrees, CW+. */
    public double getElbowPosition() {
        return elbowMotor.getEncoder().getPosition();
    }

    /** Returns the extension of the extender in inches, 0 = retracted, ceiling = extended, CCW+. */
    public double getExtenderPosition() {
        return CraneConstants.kExtenderStart - degreesToInches(extenderMotor.getEncoder().getPosition());
    }

    /** Returns the angle of the gripper in degrees, CW+. */
    public double getGripperPosition() {
        return gripperMotor.getEncoder().getPosition();
    }

    /** Returns the angle of the wrist in degrees, CCW+. */
    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }

    public double getRelativeWristPos() {
        return getWristPosition() - getElbowPosition();
    }

    /** Returns the angle of the sucker in degrees, CCW+. */
    public double getSuckerPosition() {
        return suckerMotor.getEncoder().getPosition();
    }

    public void intakeLog() {
        Logger.info("ELBOW", Double.toString(getElbowPosition()) + " Actual Degrees -> " + Double.toString(elbowSetpoint) + " Target Degrees");
        Logger.info("EXTENDER", Double.toString(getExtenderPosition()) + " Actual Inches -> " + Double.toString(extenderSetpoint) + " Target Inches");
        Logger.info("WRIST", Double.toString(getRelativeWristPos()) + " Actual Degrees -> " + Double.toString(wristSetpoint) + " Target Degrees");
        Logger.info("GRIPPER", Double.toString(getGripperPosition()) + " Actual Degrees -> " + Double.toString(gripperSetpoint) + " Target Degrees");
        if(elbowMotor.getFaults().rawBits != 0) Logger.warn("ELBOW: " + elbowMotor.getFaults().toString());
        if(extenderMotor.getFaults().rawBits != 0) Logger.warn("EXTENDER: " + extenderMotor.getFaults().toString());
        if(wristMotor.getFaults().rawBits != 0) Logger.warn("WRIST: " + wristMotor.getFaults().toString());
        if(gripperMotor.getFaults().rawBits != 0) Logger.warn("GRIPPER: " + gripperMotor.getFaults().toString());
    }
}
