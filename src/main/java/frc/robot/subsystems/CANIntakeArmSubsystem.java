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

public class CANIntakeArmSubsystem extends SubsystemBase {
    // CONSTANTS
    private final double ZERO_OFFSET = 0;
    private final double GRAVITY_COMPENSATION = 0.75;

    private final double P = 0.01;
    private final double I = 0;
    private final double D = 0;
    // END CONSTANTS

    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    // Profiled to ensure smooth movement
    private final ProfiledPIDController intakePIDController;

    private NetworkTableEntry nt_angle, nt_desiredAngle;
    private boolean isDone = false;

    public CANIntakeArmSubsystem() {
        intakeMotor = new SparkMax(56, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();

        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        intakeMotorConfig
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

        intakeMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        intakeMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(P).i(I).d(D);

        intakeMotor.configure(
            intakeMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        TrapezoidProfile.Constraints intakePIDConstraints;
        // Set both max velocity and max acceleration to 10 deg/sec
        intakePIDConstraints = new TrapezoidProfile.Constraints(10, 10);
        // Temp PID values, will probably need tuning
        intakePIDController = new ProfiledPIDController(P, I, D, intakePIDConstraints);

        nt_angle = SmartDashboard.getEntry("Intake Angle");
        nt_desiredAngle = SmartDashboard.getEntry("Intake Desired Angle");
        nt_desiredAngle.setDefaultDouble(55);
    }
    
    public double getEncoderAngle() {
        // Converting from rotations to degrees
        return (intakeEncoder.getPosition()*360)%360 - ZERO_OFFSET;
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

        double intakeMotorSetpoint = MathUtil.clamp(nt_desiredAngle.getDouble(40), 20, 60);

        double intakeMotorVoltage = /*GRAVITY_COMPENSATION * Math.cos(Math.toRadians(encoderAngle)) 
                                      +*/ intakePIDController.calculate(encoderAngle, intakeMotorSetpoint);
        
        System.out.print(intakeMotorVoltage + " ");
        System.out.print(intakeMotorSetpoint + " ");
        System.out.println(encoderAngle);

        intakeMotor.setVoltage(intakeMotorVoltage);
        isDone = Math.abs(encoderAngle - intakeMotorSetpoint) < 1;
    }
}