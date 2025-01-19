
//This will be the control for the claw on the robot
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
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


    public Claw() {
        // Initializing the Gripper motorSparkMax max = new SparkMax(1, MotorType.kBrushless);
        gripperMotor = new SparkMax(CANIDS.GRIPPER, MotorType.kBrushless);
        gripperConfig = new SparkMaxConfig();
        gripperPID = gripperMotor.getClosedLoopController();

        wristMotor = new SparkMax(CANIDS.WRIST, MotorType.kBrushless);
        wristConfig = new SparkMaxConfig();
        wristPID = wristMotor.getClosedLoopController();

        gripperConfig
            .idleMode(IdleMode.kBrake);
        gripperConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(ClawConstants.kGripperEncoderDistancePerPulse)
            .velocityConversionFactor(ClawConstants.kGripperEncoderDistancePerPulse);
        gripperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        gripperMotor.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        wristConfig
            .idleMode(IdleMode.kBrake);
        wristConfig.encoder
        // TODO: Ratio needs to be changed
            .positionConversionFactor(ClawConstants.kWristEncoderDistancePerPulse)
            .velocityConversionFactor(ClawConstants.kWristEncoderDistancePerPulse);
        wristConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.001, 0.0, 0.0);
            
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    }

    /**
     * Sets the position of the gripper relative to its starting position
     * 
     * @param setPoint The desired position of the gripper Motor
     */
    public void setGripperPosition(double setPoint) {
        gripperPID.setReference(setPoint, ControlType.kPosition);
    }
}
