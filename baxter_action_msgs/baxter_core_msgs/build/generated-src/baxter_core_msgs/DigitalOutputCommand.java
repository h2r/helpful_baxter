package baxter_core_msgs;

public interface DigitalOutputCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/DigitalOutputCommand";
  static final java.lang.String _DEFINITION = "##the name of the output\nstring name  \n##the value to set output \nbool value   \n";
  java.lang.String getName();
  void setName(java.lang.String value);
  boolean getValue();
  void setValue(boolean value);
}
