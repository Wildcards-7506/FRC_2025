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
    public boolean onClimberControl = false;

    // Rotator
    // private final SparkMax rotatorMotor;
    // private final SparkMaxConfig rotatorConfig;
    // public final SparkClosedLoopController rotatorPID;
    // public double rotatorSetpoint;
    
    // Anchor
    private final SparkMax anchorMotor1;
    private final SparkMax anchorMotor2;
    private final SparkMaxConfig anchorConfig;
    public double anchorSetpoint;

    public Climber() {
        // Rotator
        // rotatorMotor = new SparkMax(CANIDS.ROTATOR, MotorType.kBrushless);
        // rotatorConfig = new SparkMaxConfig();
        // rotatorPID = rotatorMotor.getClosedLoopController();

        // Anchor
        anchorMotor1 = new SparkMax(CANIDS.ANCHOR1, MotorType.kBrushless);
        anchorMotor2 = new SparkMax(CANIDS.ANCHOR2, MotorType.kBrushless);
        anchorConfig = new SparkMaxConfig();

        // Configure setpoints
        // setRotatorPosition(ClimberConstants.kRotatorHardDeck);
        // setAnchorPosition(ClimberConstants.kAnchorHardDeck);

        // Set the PID coefficients
        // rotatorConfig
        //     .smartCurrentLimit(80)
        //     .idleMode(IdleMode.kBrake);
        // rotatorConfig.softLimit
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimitEnabled(true)
        //     .forwardSoftLimit(ClimberConstants.kRotatorCeiling + 2)
        //     .reverseSoftLimit(ClimberConstants.kRotatorHardDeck - 2);
        // rotatorConfig.encoder
        //     .positionConversionFactor(ClimberConstants.kRotatorEncoderDistancePerPulse)
        //     .velocityConversionFactor(ClimberConstants.kRotatorEncoderDistancePerPulse);
        // rotatorConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     // TODO: PID values changed temporarily for testing
        //     .pid(0.05, 0.0, 0.0);
            
        // rotatorMotor.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        anchorConfig
            .smartCurrentLimit(100)
            .idleMode(IdleMode.kBrake);
        anchorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(ClimberConstants.kAnchorCeiling)
            .reverseSoftLimit(ClimberConstants.kAnchorHardDeck);
        anchorConfig.encoder
            .positionConversionFactor(ClimberConstants.kAnchorEncoderDistancePerPulse);
        anchorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            
        anchorMotor1.configure(anchorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        anchorMotor2.configure(anchorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        anchorSetpoint = getAnchorPosition();
        // rotatorSetpoint = getRotatorPosition();
    }

    /**
     * Sets the extension of the rotator.
     * 
     * @param setPoint The desired extension of the rotator in inches
     */
    // public void setRotatorPosition(double setPoint) {
    //     rotatorSetpoint = filterSetPoint(setPoint, 
    //                                    ClimberConstants.kRotatorHardDeck, 
    //                                    ClimberConstants.kRotatorCeiling);
    //     rotatorPID.setReference(rotatorSetpoint, ControlType.kPosition);
    //     SmartDashboard.putNumber("Rotator SetP", rotatorSetpoint);
    //     SmartDashboard.putNumber("Rotator Pos", getRotatorPosition());
    // }

    public void setAnchorVoltage(double volts) {
        anchorMotor1.setVoltage(volts);
        anchorMotor2.setVoltage(volts);
        SmartDashboard.putNumber("Anchor Pos", getAnchorPosition());
    }

    // public double getRotatorPosition() {
    //     return rotatorMotor.getEncoder().getPosition();
    // }

    public double getAnchorPosition() {
        return anchorMotor1.getEncoder().getPosition();
    }
}