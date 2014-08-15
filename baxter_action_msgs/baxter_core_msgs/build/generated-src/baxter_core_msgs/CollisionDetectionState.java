package baxter_core_msgs;

public interface CollisionDetectionState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/CollisionDetectionState";
  static final java.lang.String _DEFINITION = "std_msgs/Header header\nbool collision_state\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getCollisionState();
  void setCollisionState(boolean value);
}
