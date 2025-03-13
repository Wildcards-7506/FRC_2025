
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

public class Crane extends SubsystemBase {
    // Crane vars
    public CraneState craneState = CraneState.STOW; // stow is the starting configuration
    // private ArmFeedforward feedforward;
    /** Degree of angleMargin so that the crane can progress to the next position. */
    public static boolean climbMode = false;
    public boolean runSetpoint = false;
    
        // Wrist
        private final SparkMax wristMotor;
        private final SparkMaxConfig wristConfig;
        public final SparkClosedLoopController wristPID;
        public double wristSetpoint;
        private double prevWristDirection = 0;
        
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
        public double suckerSetpoint;
        
        public Crane() {
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
    
            wristConfig
                .smartCurrentLimit(20)
                .idleMode(IdleMode.kBrake);
            wristConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(CraneConstants.kWristCeiling)
                .reverseSoftLimit(CraneConstants.kWristHardDeck - 20);
            wristConfig.encoder
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
                .pid(0.005, 0.000003, 0.1);
                
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
                .pid(0.005, 0.0, 0.1);
                
            extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            suckerConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);
                
            suckerMotor.configure(suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            // Set up setpoints for each motor
            setWristPosition(CraneConstants.kWristHardDeck);
            setElbowPosition(CraneConstants.kElbowHardDeck);
            setExtenderPosition(CraneConstants.kExtenderStart);
        }
        
        /**
         * This method spins the sucker motor based on voltage and the direction 
         * provided by a sign (e.g. -12).
         * 
         * @param volts The volts to spin the sucker motor, max around (+/-) 12 volts.
         */
        public void spinSucker(double volts) {
            suckerMotor.setVoltage(volts);
        }
    
        public void engageClimbMode(){
            climbMode = true;
    }
    public boolean getClimbMode(){
        return climbMode;
    }

    /**
     * Sets the angle of the wrist, shaft CCW+.
     * 
     * @param setPoint The desired angle of the wrist in degrees
     */
    public void setWristPosition(double setPoint) {
        wristSetpoint = filterSetPoint(setPoint, 
                                       CraneConstants.kWristHardDeck, 
                                       CraneConstants.kWristCeiling);
        /*
         * don't let integral accumulate if more than a few degrees away from setpoint
         */
        if(Math.abs(getWristPosition() - wristSetpoint) > 10 ||
           Math.abs(getWristPosition() - wristSetpoint) < 0.25 ||
           prevWristDirection != Math.signum(getWristPosition() - wristSetpoint)) {
            wristPID.setIAccum(0.0);
        }
        prevWristDirection = Math.signum(getWristPosition() - wristSetpoint);
        setPoint = wristSetpoint;
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
                                          CraneConstants.kExtenderHardDeck-0.25, 
                                          CraneConstants.kExtenderCeiling);
        setPoint = CraneConstants.kExtenderStart - extenderSetpoint;
        setPoint = inchesToDegrees(setPoint);
        extenderPID.setReference(setPoint, ControlType.kPosition);
        SmartDashboard.putNumber("Extender SetP", extenderSetpoint);
        SmartDashboard.putNumber("Extender Pos", getExtenderPosition());
    }

    private double inchesToDegrees(double inches) {
        return inches * 360 / CraneConstants.kPulleyCircumferenceInches;
    }

    private double degreesToInches(double degrees) {
        return degrees * CraneConstants.kPulleyCircumferenceInches / 360;
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

    /** Returns the angle of the wrist in degrees, CCW+. */
    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }

    /** Returns the angle of the sucker in degrees, CCW+. */
    public double getSuckerPosition() {
        return suckerMotor.getEncoder().getPosition();
    }

    public double getSuckerCurrent() {
        return suckerMotor.getOutputCurrent();
    }
}
