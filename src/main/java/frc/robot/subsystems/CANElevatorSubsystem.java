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

    private final double P = 1.0;
    private final double I = 0;
    private final double D = 0.01;
    // END CONSTANTS

    private final SparkMax elevatorMotor;
    private final RelativeEncoder elevatorEncoder;

    // Profiled to ensure smooth movement
    private final ProfiledPIDController elevatorPIDController;

    private NetworkTableEntry nt_rotation, nt_desiredRotation;
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
        // Set both max velocity and max acceleration to 3.0 rotations/sec
        elevatorPIDConstraints = new TrapezoidProfile.Constraints(3.0, 3.0);
        // Temp PID values, will probably need tuning
        elevatorPIDController = new ProfiledPIDController(P, I, D, elevatorPIDConstraints);

        nt_rotation = SmartDashboard.getEntry("Elevator Rotation");
        nt_desiredRotation = SmartDashboard.getEntry("Elevator Desired Rotation");
        nt_desiredRotation.setDefaultDouble(1);
    }
    
    public double getEncoderRotation() {
        return elevatorEncoder.getPosition() - ZERO_OFFSET;
    }

    public void setDesiredRotation(double rotations) {
        nt_desiredRotation.setNumber(rotations);
        isDone = false;
    }

    public boolean isAtDesiredRotation() {
        return isDone;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        final double encoderRotation = getEncoderRotation();

        nt_rotation.setDouble(encoderRotation);

        double elevatorMotorSetpoint = MathUtil.clamp(nt_desiredRotation.getDouble(1), 0, 10);

        double elevatorMotorVoltage = /*GRAVITY_COMPENSATION * Math.cos(Math.toRadians(encoderRotation)) 
                                      +*/ elevatorPIDController.calculate(encoderRotation, elevatorMotorSetpoint);
        
        System.out.print(elevatorMotorVoltage + " ");
        System.out.print(elevatorMotorSetpoint + " ");
        System.out.println(encoderRotation + " ");
        System.out.println(isDone);

        elevatorMotor.setVoltage(elevatorMotorVoltage);
        isDone = Math.abs(encoderRotation - elevatorMotorSetpoint) < 0.01;
    }
}