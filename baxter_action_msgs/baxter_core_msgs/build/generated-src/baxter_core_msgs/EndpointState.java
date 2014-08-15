package baxter_core_msgs;

public interface EndpointState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/EndpointState";
  static final java.lang.String _DEFINITION = "Header header\ngeometry_msgs/Pose   pose\ngeometry_msgs/Twist  twist\ngeometry_msgs/Wrench wrench\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  geometry_msgs.Twist getTwist();
  void setTwist(geometry_msgs.Twist value);
  geometry_msgs.Wrench getWrench();
  void setWrench(geometry_msgs.Wrench value);
}
