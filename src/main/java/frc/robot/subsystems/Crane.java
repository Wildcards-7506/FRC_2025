
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;
import frc.robot.Constants.CraneState;
import frc.robot.utils.Logger;

public class Crane extends SubsystemBase {
    // Crane vars
    public CraneState craneState = CraneState.STOW; // stow is the starting configuration

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
    // public boolean wristScoreState = false;

    // Arm: Elbow and Extender

    // Elbow
    private final SparkMax elbowMotor;
    private final SparkMaxConfig elbowConfig;
    public final SparkClosedLoopController elbowPID;
    public double elbowSetpoint;
    private ArmFeedforward feedforward;
    
    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;
    public double extenderSetpoint;

    //Sucker
    private final SparkFlex suckerMotor;
    private final SparkFlexConfig suckerConfig;
    public final SparkClosedLoopController suckerPID;
    
    public Crane() {
        // Initializing the Gripper motorSparkMax max = new SparkMax(1, MotorType.kBrushless);
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

        suckerMotor = new SparkFlex(CANIDS.SUCKER, MotorType.kBrushless);
        suckerConfig = new SparkFlexConfig();
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
            .reverseSoftLimit(CraneConstants.kWristHardDeck);
        wristConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.005, 0.0, 0.1);
            
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
            // TODO: PID values changed temporarily for testing, 1/25/2025 was: 0.01, 0.01, 0.5 
            .pid(0.005, 0.0000025, 0.7);
            
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderConfig
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        extenderConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(inchesToDegrees(CraneConstants.kExtenderCeiling))
            .reverseSoftLimit(inchesToDegrees(CraneConstants.kExtenderHardDeck));
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
            .smartCurrentLimit(20)
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
    
    public void spinSucker(double velocity) {
        suckerPID.setReference(velocity, ControlType.kVelocity);
    }

    /**
     * Sets the angle of the wrist, shaft CCW+.
     * 
     * @param setPoint The desired angle of the wrist in degrees
     */
    public void setWristPosition(double setPoint) {
        if(craneState == CraneState.HIGH_REEF) {
            wristSetpoint = filterSetPoint(setPoint, 
                                           CraneConstants.kWristHigh, 
                                           CraneConstants.kWristHardDeck);
        } else {
            wristSetpoint = filterSetPoint(setPoint, 
                                        CraneConstants.kWristHardDeck, 
                                        CraneConstants.kWristCeiling);
        }
        setPoint = getElbowPosition() + wristSetpoint;
        wristPID.setReference(setPoint, ControlType.kPosition);
        SmartDashboard.putNumber("Wrist SetP", wristSetpoint);
        SmartDashboard.putNumber("Wrist Pos", getWristPosition());
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
         * don't let integral accumulate if more than 10 degrees away
         */
        if(Math.abs(getElbowPosition() - elbowSetpoint) > 13) {
            elbowPID.setIAccum(0.0);
        }
        // System.out.println("Integral Accum: " + elbowPID.getIAccum());
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
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

    public void intakeLog() {
        Logger.info("ELBOW", Double.toString(getElbowPosition()) + " Actual Degrees -> " + Double.toString(elbowSetpoint) + " Target Degrees");
        Logger.info("EXTENDER", Double.toString(getExtenderPosition()) + " Actual Inches -> " + Double.toString(extenderSetpoint) + " Target Inches");
        Logger.info("WRIST", Double.toString(getWristPosition()) + " Actual Degrees -> " + Double.toString(wristSetpoint) + " Target Degrees");
        Logger.info("GRIPPER", Double.toString(getGripperPosition()) + " Actual Degrees -> " + Double.toString(gripperSetpoint) + " Target Degrees");
        if(elbowMotor.getFaults().rawBits != 0) Logger.warn("ELBOW: " + elbowMotor.getFaults().toString());
        if(extenderMotor.getFaults().rawBits != 0) Logger.warn("EXTENDER: " + extenderMotor.getFaults().toString());
        if(wristMotor.getFaults().rawBits != 0) Logger.warn("WRIST: " + wristMotor.getFaults().toString());
        if(gripperMotor.getFaults().rawBits != 0) Logger.warn("GRIPPER: " + gripperMotor.getFaults().toString());
    }
}
