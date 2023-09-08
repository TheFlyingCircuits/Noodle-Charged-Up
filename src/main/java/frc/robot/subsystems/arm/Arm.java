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

    private PIDController velocityController;
    private ArmFeedforward feedforward;
    private TrapezoidProfile trapezoidProfile;

    private final Timer timer;
    private double lastTime;

    private boolean isMovingToTarget;
    private double setpointDegrees;


    private TrapezoidProfile.Constraints motionConstraints;

    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmIOInputsAutoLogged();

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

        this.motionConstraints = new TrapezoidProfile.Constraints(
            Constants.Arm.maxDesiredVelocityDegreesPerSecond,
            Constants.Arm.maxDesiredAccelerationDegreesPerSecond);
    }

    public void setArmPositionDegrees(double setpointDegrees) {
        setpointDegrees = MathUtil.clamp(setpointDegrees, Constants.Arm.minAngleDegrees, Constants.Arm.maxAngleDegrees);
        this.setpointDegrees = setpointDegrees;

        trapezoidProfile = new TrapezoidProfile(
            motionConstraints,
            new TrapezoidProfile.State(
                setpointDegrees, 0.0
            ),
            new TrapezoidProfile.State(
                inputs.armPosition.getDegrees(), inputs.armRotateDegreesPerSecond
            ));

        timer.reset();
        timer.start();

        isMovingToTarget = true;
    }

    public void setArmPosition(ArmPosition position) {
        switch (position) {
            case Intake:
                setpointDegrees = Constants.Arm.intakePositionRadians;
                break;
            case Low:
                setpointDegrees = Constants.Arm.lowPositionRadians;
                break;
            case Mid:
                setpointDegrees = Constants.Arm.midPositionRadians;
                break;
            case High:
                setpointDegrees = Constants.Arm.highPositionRadians;
                break;
            case Default:
            default:
                setpointDegrees = Constants.Arm.defaultPositionRadians;
                break;
        }
        setArmPositionDegrees(setpointDegrees);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if (inputs.atFrontLimitSwitch || inputs.atBackLimitSwitch) {
            io.setArmVoltage(0.0);
        } else {
            io.setArmVoltage(
                feedforward.calculate(currentProfilePosition, currentProfileVelocity)
                
                );
        }
    }
}
