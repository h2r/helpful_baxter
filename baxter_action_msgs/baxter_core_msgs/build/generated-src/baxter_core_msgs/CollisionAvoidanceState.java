package baxter_core_msgs;

public interface CollisionAvoidanceState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/CollisionAvoidanceState";
  static final java.lang.String _DEFINITION = "std_msgs/Header header\nbool other_arm\nstring[] collision_object";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getOtherArm();
  void setOtherArm(boolean value);
  java.util.List<java.lang.String> getCollisionObject();
  void setCollisionObject(java.util.List<java.lang.String> value);
}
