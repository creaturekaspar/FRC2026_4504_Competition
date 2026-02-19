package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: add constants to the constants file, rather than magic numbers or constants within this class

public class CANElevatorSubsystem extends SubsystemBase {
    // CONSTANTS
    private final double ZERO_OFFSET = 0;
    private final double GRAVITY_COMPENSATION = 0.75;

    private final double P = 0.3;
    private final double I = 0;
    private final double D = 0;
    // END CONSTANTS

    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;

    // Profiled to ensure smooth movement
    private final ProfiledPIDController elevatorPIDController;

    private NetworkTableEntry nt_angle, nt_desiredAngle;
    private boolean isDone = false;

    public CANElevatorSubsystem() {
        elevatorMotor = new SparkMax(51, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

        elevatorMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

        elevatorMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        elevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P).i(I).d(D);

        elevatorMotor.configure(
            elevatorMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        TrapezoidProfile.Constraints elevatorPIDConstraints;
        // Set both max velocity and max acceleration to 45 deg/sec
        elevatorPIDConstraints = new TrapezoidProfile.Constraints(45, 45);
        // Temp PID values, will probably need tuning
        elevatorPIDController = new ProfiledPIDController(P, I, D, elevatorPIDConstraints);

        nt_angle = SmartDashboard.getEntry("Elevator Angle");
        nt_desiredAngle = SmartDashboard.getEntry("Elevator Desired Angle");
        nt_desiredAngle.setDefaultDouble(55);
    }
    
    public double getEncoderAngle() {
        // Converting from rotations to degrees
        return -elevatorEncoder.getPosition()*360 - ZERO_OFFSET;
    }

    public void setDesiredAngle(double degrees) {
        nt_desiredAngle.setNumber(degrees);
        isDone = false;
    }

    public boolean isAtDesiredAngle() {
        return isDone;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        final double encoderAngle = getEncoderAngle();

        nt_angle.setDouble(encoderAngle);

        double elevatorMotorSetpoint = MathUtil.clamp(nt_desiredAngle.getDouble(50), 20, 60);

        double elevatorMotorVoltage = GRAVITY_COMPENSATION * Math.cos(Math.toRadians(encoderAngle)) 
                                      + elevatorPIDController.calculate(encoderAngle, elevatorMotorSetpoint);
                                
        elevatorMotor.setVoltage(elevatorMotorVoltage);
        isDone = Math.abs(encoderAngle - elevatorMotorSetpoint) < 1;
    }
}