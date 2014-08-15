package baxter_core_msgs;

public interface OpenCameraRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/OpenCameraRequest";
  static final java.lang.String _DEFINITION = "string          name\nCameraSettings  settings\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  baxter_core_msgs.CameraSettings getSettings();
  void setSettings(baxter_core_msgs.CameraSettings value);
}
