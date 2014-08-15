package baxter_core_msgs;

public interface JointCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/JointCommand";
  static final java.lang.String _DEFINITION = "int32 mode\nfloat64[] command\nstring[]  names\n\nint32 POSITION_MODE=1\nint32 VELOCITY_MODE=2\nint32 TORQUE_MODE=3\nint32 RAW_POSITION_MODE=4";
  static final int POSITION_MODE = 1;
  static final int VELOCITY_MODE = 2;
  static final int TORQUE_MODE = 3;
  static final int RAW_POSITION_MODE = 4;
  int getMode();
  void setMode(int value);
  double[] getCommand();
  void setCommand(double[] value);
  java.util.List<java.lang.String> getNames();
  void setNames(java.util.List<java.lang.String> value);
}
