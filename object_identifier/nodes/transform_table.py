import roslib
roslib.load_manifest("object_identifier")
import rospy
import tf
import tf.transformations
from object_recognition_msgs.msg import TableArray, Table
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from baxter_pick_and_place.move_helper import MoveHelper

class TableTransformer:
	def __init__(self):
		self.transformer = tf.TransformListener()
		rospy.Subscriber("table_array", TableArray, self.table_callback)
		self.publisher = rospy.Publisher("table_array_transformed", TableArray)

	def table_callback(self, msg):
		now = rospy.Time.now()
		self.transformer.waitForTransform("world", msg.header.frame_id, now, rospy.Duration(4.0))
		table_array = TableArray()
		table_array.header.frame_id = "world"
		highest_table = None
		for table in msg.tables:
			new_table = Table()
			new_table.header.frame_id = "world"
			table_pose = PoseStamped(pose=table.pose, header = table.header)
			new_table.pose = self.transformer.transformPose("world", table_pose).pose
			new_table.convex_hull = table.convex_hull
			table_array.tables.append(new_table)

			if highest_table is None or new_table.pose.position.z > highest_table.pose.position.z:
				highest_table = new_table

		self.publisher.publish(table_array)
		MoveHelper.add_table(position=highest_table.pose.position)



if __name__ == "__main__":
	global transformer, publisher
	rospy.init_node("table_transformer")
	transformer = TableTransformer()
	
	rospy.spin()