package baxter_core_msgs;

public interface SolvePositionIKRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/SolvePositionIKRequest";
  static final java.lang.String _DEFINITION = "# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.\ngeometry_msgs/PoseStamped[] pose_stamp\n\n# (optional) Joint Angle Seed(s) for IK solver.\n# * specify a JointState seed for each pose_stamp, using name[] and position[]\n# * empty arrays or a non-default seed_mode will cause user seed to not be used\nsensor_msgs/JointState[] seed_angles\n\n# Seed Type Mode\n# * default (SEED_AUTO) mode: iterate through seed types until first valid\n#                             solution is found\n# * setting any other mode:   try only that seed type\nuint8 SEED_AUTO    = 0\nuint8 SEED_USER    = 1\nuint8 SEED_CURRENT = 2\nuint8 SEED_NS_MAP  = 3\n\nuint8 seed_mode\n";
  static final byte SEED_AUTO = 0;
  static final byte SEED_USER = 1;
  static final byte SEED_CURRENT = 2;
  static final byte SEED_NS_MAP = 3;
  java.util.List<geometry_msgs.PoseStamped> getPoseStamp();
  void setPoseStamp(java.util.List<geometry_msgs.PoseStamped> value);
  java.util.List<sensor_msgs.JointState> getSeedAngles();
  void setSeedAngles(java.util.List<sensor_msgs.JointState> value);
  byte getSeedMode();
  void setSeedMode(byte value);
}
