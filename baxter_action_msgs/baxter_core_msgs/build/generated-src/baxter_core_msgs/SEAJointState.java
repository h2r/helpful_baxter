package baxter_core_msgs;

public interface SEAJointState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/SEAJointState";
  static final java.lang.String _DEFINITION = "# This is a message that holds data to describe the state of a set of torque controlled joints.\n#\n# The state of each joint (revolute or prismatic) is defined by:\n#  * the position of the joint (rad or m),\n#  * the velocity of the joint (rad/s or m/s) and\n#  * the effort that is applied in the joint (Nm or N).\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state.\n# The goal is to make each of the fields optional. When e.g. your joints have no\n# effort associated with them, you can leave the effort array empty.\n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\n\nHeader header\n\nstring[] name\nfloat64[] commanded_position\nfloat64[] commanded_velocity\nfloat64[] commanded_effort\nfloat64[] actual_position\nfloat64[] actual_velocity\nfloat64[] actual_effort\nfloat64[] gravity_model_effort\nfloat64[] hysteresis_model_effort\nfloat64[] crosstalk_model_effort\nfloat64   hystState";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getName();
  void setName(java.util.List<java.lang.String> value);
  double[] getCommandedPosition();
  void setCommandedPosition(double[] value);
  double[] getCommandedVelocity();
  void setCommandedVelocity(double[] value);
  double[] getCommandedEffort();
  void setCommandedEffort(double[] value);
  double[] getActualPosition();
  void setActualPosition(double[] value);
  double[] getActualVelocity();
  void setActualVelocity(double[] value);
  double[] getActualEffort();
  void setActualEffort(double[] value);
  double[] getGravityModelEffort();
  void setGravityModelEffort(double[] value);
  double[] getHysteresisModelEffort();
  void setHysteresisModelEffort(double[] value);
  double[] getCrosstalkModelEffort();
  void setCrosstalkModelEffort(double[] value);
  double getHystState();
  void setHystState(double value);
}
