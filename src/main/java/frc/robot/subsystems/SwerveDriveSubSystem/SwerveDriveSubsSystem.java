package frc.robot.subsystems.SwerveDriveSubSystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class SwerveDriveSubsSystem extends SubsystemBase {





  /** AdvantageKit inputs for the swerve drive */
  @AutoLog
  public static class SwerveInputs {
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    public SwerveModuleState[] states = new SwerveModuleState[4];
    public Angle gyroRotation = Degrees.of(0);
    public ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    public Pose2d estimatedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveDrive drive;
    private final Field2d field = new Field2d();
    private final Supplier<Angle> gyroAngleSupplier;


      private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(12.75))  // MK4i L1 ratio
        .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / (MetersPerSecond.of(1).in(MetersPerSecond) / Inches.of(4).in(Meters)), 0.01))
        .withStatorCurrentLimit(Amps.of(40))        // prevents belt slip
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
}   
     private SmartMotorControllerConfig buildAzimuthCfg() {
    return new SmartMotorControllerConfig(this)
        .withClosedLoopController(1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 1))
        .withGearing(new MechanismGearing(6.75))   // MK4i steer ratio
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
  }
   public SwerveModule createModule(TalonFX drive, TalonFX azimuth,
                                   CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location) {
    SmartMotorController driveSMC   = new TalonFXWrapper(drive,   DCMotor.getKrakenX60(1), buildDriveCfg());
    SmartMotorController azimuthSMC = new TalonFXWrapper(azimuth, DCMotor.getFalcon500(1), buildAzimuthCfg());

    return new SwerveModule(new SwerveModuleConfig(driveSMC, azimuthSMC)
        // CANcoder eliminates the need to home the steer motor at startup.
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
        .withLocation(location)
        // State optimization rotates the module at most 90 deg instead of 180 deg + reversing drive.
        .withOptimization(true));
  }

  public SwerveDriveSubsSystem(){
     Pigeon2 gyro = new Pigeon2(14);
     gyroAngleSupplier = gyro.getYaw().asSupplier();

    // Module locations: +X forward, +Y left. 24-inch offsets assume module
    // centers are 24 in from the robot center, update to match your chassis.
    // CAN IDs are grouped as (drive, steer, CANcoder) per module.
    SwerveModule fl = createModule(new TalonFX(1),
                          new TalonFX(2),
                          new CANcoder(3), "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    SwerveModule fr = createModule(new TalonFX(4),
                          new TalonFX(5),
                          new CANcoder(6), "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    SwerveModule bl = createModule(new TalonFX(7),
                          new TalonFX(8 ),
                          new CANcoder(9), "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    SwerveModule br = createModule(new TalonFX(10 ),
                          new TalonFX(11 ),
                          new CANcoder(12), "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));

    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        // gyro.getYaw() gives the heading used for field-relative driving.
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        // Translation and rotation PIDs are used by driveToPose(); kP=1 is a conservative start.
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
    SmartDashboard.putData("Field", field);

  }
  private Rotation2d getGyroAngle() {
    return new Rotation2d(swerveInputs.gyroRotation);
  }

  private void updateInputs() {
    swerveInputs.estimatedPose = drive.getPose();
    swerveInputs.states = drive.getModuleStates();
    swerveInputs.positions = drive.getModulePositions();
    swerveInputs.robotRelativeSpeeds = drive.getRobotRelativeSpeed();
    swerveInputs.gyroRotation = gyroAngleSupplier.get();
  }
  public Command setRobotRelativeChassisSpeeds(Supplier<ChassisSpeeds> speedsSupplier) {
    return run(() -> {
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speedsSupplier.get());
      SwerveModuleState[] states = drive.getStateFromRobotRelativeChassisSpeeds(speedsSupplier.get());
      Logger.recordOutput("Swerve/DesiredStates", states);
      drive.setSwerveModuleStates(states);
    }).withName("Set Robot Relative Chassis Speeds");
  }

   /**
   * Drive the robot with field-relative chassis speeds.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
        .withName("Field Oriented Drive");
  }

  /** Drive to a specific field pose. */
  public Command driveToPose(Pose2d pose) { return drive.driveToPose(pose); }

  /** Lock wheels in X pattern to resist pushing. */
  public Command lock() { return run(drive::lockPose); }

  public Pose2d getPose() { return swerveInputs.estimatedPose; }
  public ChassisSpeeds getRobotRelativeSpeeds() { return swerveInputs.robotRelativeSpeeds; }  

  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationX,
                                                     DoubleSupplier translationY,
                                                     DoubleSupplier rotation) {
    return new SwerveInputStream(drive, translationX, translationY, rotation)
        .withMaximumAngularVelocity(DegreesPerSecond.of(360))
        .withMaximumLinearVelocity(MetersPerSecond.of(1))
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()     // non-linear rotation response
        .withCubeTranslationControllerAxis()  // non-linear translation response
        .withAllianceRelativeControl();        // alliance-aware field-relative
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();           // updates pose estimator and telemetry
    field.setRobotPose(drive.getPose());
    updateInputs();
    Logger.processInputs("Swerve", swerveInputs); // keeps the Field2d widget current
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();  // steps the motor physics simulation
  }

}
