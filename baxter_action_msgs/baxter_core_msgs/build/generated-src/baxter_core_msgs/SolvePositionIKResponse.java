package baxter_core_msgs;

public interface SolvePositionIKResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/SolvePositionIKResponse";
  static final java.lang.String _DEFINITION = "# joints[i]      == joint angle solution for each pose_state[i]\nsensor_msgs/JointState[] joints\n\n# NOTE: isValid will be deprecated by result_type in future versions\nbool[] isValid\n\n# result_type[i] == seed type used to find valid solution, joints[i];\n# otherwise,     == RESULT_INVALID (no valid solution found).\nuint8 RESULT_INVALID = 0\nuint8[] result_type";
  static final byte RESULT_INVALID = 0;
  java.util.List<sensor_msgs.JointState> getJoints();
  void setJoints(java.util.List<sensor_msgs.JointState> value);
  boolean[] getIsValid();
  void setIsValid(boolean[] value);
  org.jboss.netty.buffer.ChannelBuffer getResultType();
  void setResultType(org.jboss.netty.buffer.ChannelBuffer value);
}
