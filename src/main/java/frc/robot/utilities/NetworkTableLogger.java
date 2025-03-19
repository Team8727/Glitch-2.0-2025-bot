package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Custom NetworkTable logger created by Glitch 2.0 in 2025. This logger is used to log values to
 * the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
 *
 * @param subsystemFor the subsystem this logger will log values for
 */
public class NetworkTableLogger {
  // NetworkTable to log to
  private final NetworkTable table;

  // Publishers for each supported type
  private final ConcurrentHashMap<String, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, IntegerPublisher> integerPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StringPublisher> stringPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<Pose2d>> pose2dPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<Pose3d>> pose3dPublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructArrayPublisher<SwerveModuleState>> swerveModuleStatePublishers = new ConcurrentHashMap<>();
  private final ConcurrentHashMap<String, StructPublisher<ChassisSpeeds>> chassisSpeedsPublishers = new ConcurrentHashMap<>();
  
  // For Field2d and other Sendable support
  private static final Map<String, Sendable> tablesToData = new HashMap<>();

  /**
   * Custom NetworkTable logger created by Glitch 2.0 in 2025. This logger is used to log values to
   * the network table (can be seen using AdvantageScope, Glass, Elastic, etc.)
   *
   * @param subsystemFor the subsystem this logger will log values for
   */
  public NetworkTableLogger(String subsystemFor) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within the default instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called what the parameter
    // subsystemFor holds: (the subsystem to log for).
    table = inst.getTable(subsystemFor);
  }

  /**
   * Get the network table that the logger is logging to
   *
   * @return the network table
   */
  public NetworkTable getNetworkTable() {
    return table;
  }

  /**
   * Log method for logging a double to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (double) that will be logged
   */
  public void logDouble(String key, double value) {
    if (!doublePublishers.containsKey(key)) {
      doublePublishers.put(key, table.getDoubleTopic(key).publish());
    }

    doublePublishers.get(key).set(value);
  }

    /**
   * Log method for logging an integer to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (int) that will be logged
   */
  public void logInt(String key, int value) {
    if (!integerPublishers.containsKey(key)) {
      integerPublishers.put(key, table.getIntegerTopic(key).publish());
    }

    integerPublishers.get(key).set(value);
  }

  /**
   * Log method for logging a boolean to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param value the value (boolean) that will be logged
   */
  public void logBoolean(String key, boolean value) {
    if (!booleanPublishers.containsKey(key)) {
      booleanPublishers.put(key, table.getBooleanTopic(key).publish());
    }

    booleanPublishers.get(key).set(value);
  }

  /**
   * Log method for logging a string to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param string the value (string) that will be logged
   */
  public void logString(String key, String value) {
    if (!stringPublishers.containsKey(key)) {
      stringPublishers.put(key, table.getStringTopic(key).publish());
    }

    stringPublishers.get(key).set(value);
  }

  /**
   * Log method for logging a Pose2d to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param pose2d the value (Pose2d) that will be logged
   */
  public void logPose2d(String key, Pose2d pose2d) {
    if (!pose2dPublishers.containsKey(key)) {
      pose2dPublishers.put(key, table.getStructTopic(key, Pose2d.struct).publish());
    }

    pose2dPublishers.get(key).set(pose2d);
  }

  /**
   * Log method for logging a Pose3d to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param pose3d the value (Pose3d) that will be logged
   */
  public void logPose3d(String key, Pose3d pose3d) {
    if (!pose3dPublishers.containsKey(key)) {
      pose3dPublishers.put(key, table.getStructTopic(key, Pose3d.struct).publish());
    }

    pose3dPublishers.get(key).set(pose3d);
  }

  /**
   * Log method for logging the ServeModuleStates to the network table (can be seen using
   * AdvantageScope, Glass, Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param swerveModuleStateList the value (SwerveModuleState[]) that will be logged
   */
  public void logSwerveModuleState(String key, SwerveModuleState[] swerveModuleStateList) {
    if (!swerveModuleStatePublishers.containsKey(key)) {
      swerveModuleStatePublishers.put(key, table.getStructArrayTopic(key, SwerveModuleState.struct).publish());
    }

    swerveModuleStatePublishers.get(key).set(swerveModuleStateList);
  }

  /**
   * Log method for logging a ChassisSpeeds to the network table (can be seen using AdvantageScope, Glass,
   * Elastic, etc.)
   *
   * @param key the key, a string, that will represent the value
   * @param chassisSpeeds the value (ChassisSpeeds) that will be logged
   */
  public void logChassisSpeeds(String key, ChassisSpeeds chassisSpeeds) {
    if (!chassisSpeedsPublishers.containsKey(key)) {
      chassisSpeedsPublishers.put(key, table.getStructTopic(key, ChassisSpeeds.struct).publish());
    }

    chassisSpeedsPublishers.get(key).set(chassisSpeeds);
  }

  /**
   * Logs a Field2d to the network table.
   *
   * @param key the key, a string, that will represent the value
   * @param field2d the value (Field2d) that will be logged
   */
  public void logField2d(String key, Field2d field2d) {
    if (!table.containsKey(key)) {
      Sendable sddata = tablesToData.get(key);
      if (sddata == null || sddata != field2d) {
        tablesToData.put(key, field2d);
        NetworkTable dataTable = table.getSubTable(key);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(field2d, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
      }
    }

    for (Sendable data : tablesToData.values()) {
      SendableRegistry.update(data);
    }
  }
}
