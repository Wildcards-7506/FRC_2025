
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.fasterxml.jackson.core.util.Separators;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CraneConstants;

public class Crane extends SubsystemBase {
    // private final AbsoluteEncoder m_turningEncoder;
    // Gripper
    // private final RelativeEncoder gripperEncoder;
    private final SparkMax gripperMotor;
    private final SparkMaxConfig gripperConfig;
    public final SparkClosedLoopController gripperPID;
    public double gripperSetpoint;
    public boolean gripState = true; // gripToggle = true: means gripping coral, false: means open

    // Wrist
    private final SparkMax wristMotor;
    private final SparkMaxConfig wristConfig;
    public final SparkClosedLoopController wristPID;
    public double wristSetpoint;
    public boolean wristScoreState = false;

    // Arm: Elbow and Extender

    // Elbow
    private final SparkMax elbowMotor;
    private final SparkMaxConfig elbowConfig;
    public final SparkClosedLoopController elbowPID;
    public double elbowSetpoint;

    //Extender
    private final SparkMax extenderMotor;
    private final SparkMaxConfig extenderConfig;
    public final SparkClosedLoopController extenderPID;
    public double extenderSetpoint;

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

        gripperConfig
            .idleMode(IdleMode.kBrake);
        gripperConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kGripperEncoderDistancePerPulse);
        gripperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        gripperMotor.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        wristConfig
            .idleMode(IdleMode.kBrake);
        wristConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elbowConfig
            .idleMode(IdleMode.kBrake);
        elbowConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kElbowEncoderDistancePerPulse);
        elbowConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        elbowMotor.configure(elbowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        extenderConfig
            .idleMode(IdleMode.kBrake);
        extenderConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse)
            .velocityConversionFactor(CraneConstants.kExtenderEncoderDistancePerPulse);
        extenderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        extenderMotor.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the position of the gripper relative to its starting position
     * 
     * @param setPoint The desired position of the gripper Motor
     */
    public void setGripperPosition(double setPoint) {
        gripperSetpoint = setPoint;
        gripperPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the position of the wrist relative to its starting position
     * 
     * @param setPoint The desired position of the wrist motor
     */
    public void setWristPosition(double setPoint) {
        wristSetpoint = setPoint;
        wristPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the position of the elbow relative to its starting position
     * 
     * @param setPoint The desired position of the elbow motor
     */
    public void setElbowPosition(double setPoint) {
        elbowSetpoint = setPoint;
        elbowPID.setReference(setPoint, ControlType.kPosition);
    }

    /**
     * Sets the position of the extender relative to its starting position
     * 
     * @param setPoint The desired position of the extender motor
     */
    public void setExtenderPosition(double setPoint) {
        extenderSetpoint = setPoint;
        extenderPID.setReference(setPoint, ControlType.kPosition);
    }
}
