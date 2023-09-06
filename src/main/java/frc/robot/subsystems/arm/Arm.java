package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    public enum ArmPosition {
        Intake,
        Low,
        Mid,
        High,
        Default
    }

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs;

    private final PIDController velocityController;
    private final ArmFeedforward feedforward;

    private final Timer timer;
    private double lastTime;

    private boolean isMovingToTarget;
    private boolean armAtDefault;

    private double setpointRadians;
    private double targetPositionRadians;
    private double desiredVelocity;
    private double currentProfilePosition;
    private double currentProfileVelocity;
    private final double maxVelocity;
    private final double maxAcceleration;
    private double targetPosition;

    private TrapezoidProfile.Constraints motionConstraints;
    private TrapezoidProfile.State lastProfiledReference;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();
        this.armAtDefault = false;

        this.velocityController = new PIDController(
                Constants.Arm.Kp,
                Constants.Arm.Ki,
                Constants.Arm.Kd);

        this.feedforward = new ArmFeedforward(
                Constants.Arm.Ks,
                Constants.Arm.Kg,
                Constants.Arm.Kv,
                Constants.Arm.Ka);

        this.timer = new Timer();
        this.lastTime = 0.0;

        this.maxVelocity = Constants.Arm.maxDesiredVelocityRadiansPerSecond;
        this.maxAcceleration = Constants.Arm.maxDesiredAccelerationRadiansPerSecond;

        this.setpointRadians = 0;
        this.desiredVelocity = 0;
        this.targetPosition = 0.0;
        this.currentProfilePosition = 0.0;
        this.currentProfileVelocity = 0.0;

        this.motionConstraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        this.lastProfiledReference = new TrapezoidProfile.State(0, 0);
    }

    public void setArmPositionRadians(double radians) {
        this.setpointRadians = MathUtil.clamp(radians, Constants.Arm.minAngleDegrees, Constants.Arm.maxAngleDegrees);
    }

    public void setArmPosition(ArmPosition position) {
        switch (position) {
            case Intake:
                setpointRadians = Constants.Arm.intakePositionRadians;
                break;
            case Low:
                setpointRadians = Constants.Arm.lowPositionRadians;
                break;
            case Mid:
                setpointRadians = Constants.Arm.midPositionRadians;
                break;
            case High:
                setpointRadians = Constants.Arm.highPositionRadians;
                break;
            case Default:
            default:
                setpointRadians = Constants.Arm.defaultPositionRadians;
                break;
        }
        lastProfiledReference = new TrapezoidProfile.State(currentProfilePosition, currentProfileVelocity);
    }

    public void updateMotionProfile() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;

        TrapezoidProfile profile = new TrapezoidProfile(
            motionConstraints,
            new TrapezoidProfile.State(targetPosition, 0),
            lastProfiledReference
        );

        TrapezoidProfile.State newState = profile.calculate(dt);
        currentProfilePosition = newState.position;
        currentProfileVelocity = newState.velocity;

        lastProfiledReference = newState;
        lastTime = currentTime;
    }

    @Override
    public void periodic() {
        if (inputs.atFrontLimitSwitch || inputs.atBackLimitSwitch) {
            io.setArmVoltage(0.0);
        } else {
            io.setArmSetpointRadians(setpointRadians);
        }
    }
}
