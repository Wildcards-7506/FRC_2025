package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    // Crane vars
    public int climberState = 0; // 0 is stow/default/starting configuration

    // Rotator
    private final SparkMax rotatorMotor;
    private final SparkMaxConfig rotatorConfig;
    public final SparkClosedLoopController rotatorPID;
    public double rotatorSetpoint;
    
    // Anchor
    private final SparkMax anchorMotor;
    private final SparkMaxConfig anchorConfig;
    public final SparkClosedLoopController anchorPID;
    public double anchorSetpoint;

    public Climber() {
        // Rotator
        rotatorMotor = new SparkMax(CANIDS.ROTATOR, MotorType.kBrushless);
        rotatorConfig = new SparkMaxConfig();
        rotatorPID = rotatorMotor.getClosedLoopController();

        // Anchor
        anchorMotor = new SparkMax(CANIDS.ANCHOR, MotorType.kBrushless);
        anchorConfig = new SparkMaxConfig();
        anchorPID = anchorMotor.getClosedLoopController();

        // Configure setpoints
        setRotatorPosition(ClimberConstants.kRotatorHardDeck);
        setAnchorPosition(ClimberConstants.kAnchorHardDeck);

        // Set the PID coefficients
        rotatorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        rotatorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kRotatorCeiling)
            .reverseSoftLimit(ClimberConstants.kRotatorHardDeck);
        rotatorConfig.encoder
            .positionConversionFactor(ClimberConstants.kRotatorEncoderDistancePerPulse)
            .velocityConversionFactor(ClimberConstants.kRotatorEncoderDistancePerPulse);
        rotatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing
            .pid(0.005, 0.0, 0.0);
            
        rotatorMotor.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        anchorConfig
            .idleMode(IdleMode.kBrake);
        anchorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(inchesToDegrees(ClimberConstants.kAnchorCeiling))
            .reverseSoftLimit(inchesToDegrees(ClimberConstants.kAnchorHardDeck));
        anchorConfig.encoder
            .positionConversionFactor(ClimberConstants.kAnchorEncoderDistancePerPulse)
            .velocityConversionFactor(ClimberConstants.kAnchorEncoderDistancePerPulse);
        anchorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: PID values changed temporarily for testing
            .pid(0.005, 0.0, 0.1);
            
        anchorMotor.configure(anchorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the extension of the rotator.
     * 
     * @param setPoint The desired extension of the rotator in inches
     */
    public void setRotatorPosition(double setPoint) {
        rotatorSetpoint = filterSetPoint(setPoint, 
                                       ClimberConstants.kRotatorHardDeck, 
                                       ClimberConstants.kRotatorCeiling);
        System.out.println(rotatorSetpoint);
        rotatorPID.setReference(rotatorSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Rotator Setpoint", rotatorSetpoint);
    }

    /**
     * Sets the extension of the Anchor.
     * 
     * @param setPoint The desired extension of the Anchor in inches
     */
    public void setAnchorPosition(double setPoint) {
        anchorSetpoint = filterSetPoint(setPoint, 
                                       ClimberConstants.kAnchorHardDeck, 
                                       ClimberConstants.kAnchorCeiling);
        System.out.println(anchorSetpoint);
        anchorPID.setReference(anchorSetpoint, ControlType.kPosition);
        SmartDashboard.putNumber("Anchor Setpoint", anchorSetpoint);
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

    // TODO: Figure out conversion ratio in-person
    private double inchesToDegrees(double inches) {
        return inches;
    }
}