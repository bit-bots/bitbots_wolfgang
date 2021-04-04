#include <wolfgang_ik/ik.h>

std::ostream &operator<<(std::ostream &out, const Eigen::Quaterniond &quat) {
  Eigen::Vector3d euler = Eigen::Matrix3d(quat).eulerAngles(0, 1, 2);
  out << "Rotation: " << euler.x() << ", " << euler.y() << ", " << euler.z() << std::endl
      <<"Rotation (q): " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
  return out;
}

std::ostream &operator<<(std::ostream &out, const Eigen::Isometry3d &iso) {
  Eigen::Vector3d euler = iso.rotation().eulerAngles(0, 1, 2);
  Eigen::Quaterniond quat(iso.rotation());
  out << "Translation: " << iso.translation().x() << ", " << iso.translation().y() << ", " << iso.translation().z()
      << std::endl
      << "Rotation: " << euler.x() << ", " << euler.y() << ", " << euler.z() << std::endl
      << "Rotation (q): " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
  return out;
}

namespace wolfgang_ik {
IK::IK() : robot_model_loader_("robot_description", false) {
  // get robot model state
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state_.reset(new robot_state::RobotState(kinematic_model));
  robot_state_->updateLinkTransforms();
  // compute the necessary link length and save them as class variables
}

bool IK::solve(Eigen::Isometry3d &l_sole_goal, robot_state::RobotStatePtr goal_state) {
  // some naming conventions:
  // hip_RY_intersect: the point where the HipYaw and HipRoll axis intersect
  // ankle_intersect: the point where AnklePitch and AnkleRoll intersect

  // the provided goal points from base_link to sole link
  // first get rid of the static transform between sole and ankle_intersect
  // vector rotate with goal rotation and subtract

  // in our robot, the chain is (alternating link -> joint -> link...), fixed joints are denoted (f)
  // base_link -> base_link_to_base(f) -> torso -> LHipYaw -> l_hip_1 -> LHipRoll -> l_hip_2 -> LHipPitch ->
  // l_upper_leg -> LKnee -> l_lower_leg -> LAnklePitch -> l_ankle -> LAnkleRoll -> l_foot -> l_sole_frame(f) -> l_sole_goal
  // link -> joint transform is always zero, joint -> link is the interesting one

  for (auto &joint : goal_state->getJointModelGroup("All")->getJointModels()) {
    double zero = 0;
    goal_state->setJointPositions(joint, &zero);
  }
  Eigen::Isometry3d l_sole_to_l_foot =
      goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_foot");
  Eigen::Isometry3d l_sole_to_l_ankle =
      goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  // calc ankle_intersection
  Eigen::Vector3d ankle_roll_axis = l_sole_to_l_foot.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnkleRoll"))->getAxis();
  Eigen::Vector3d ankle_pitch_axis = l_sole_to_l_ankle.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnklePitch"))->getAxis();
  Eigen::Vector3d ankle_intersection_point;
  bool success = findIntersection(l_sole_to_l_foot.translation(),
                                  ankle_roll_axis,
                                  l_sole_to_l_ankle.translation(),
                                  ankle_pitch_axis,
                                  ankle_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d ankle_intersection = l_sole_goal;
  ankle_intersection.translation() += ankle_intersection_point;

  // get rid of static transform between base_link and hip_RY_intersect
  Eigen::Isometry3d base_link_to_hip_1 = goal_state->getGlobalLinkTransform("l_hip_1");
  Eigen::Isometry3d base_link_to_hip_2 = goal_state->getGlobalLinkTransform("l_hip_2");
  // calc ankle_intersection
  Eigen::Vector3d hip_yaw_axis = base_link_to_hip_1.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipYaw"))->getAxis();
  Eigen::Vector3d hip_roll_axis = base_link_to_hip_2.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipRoll"))->getAxis();
  Eigen::Vector3d hip_ry_intersection_point;
  success = findIntersection(base_link_to_hip_1.translation(),
                             hip_yaw_axis,
                             base_link_to_hip_2.translation(),
                             hip_roll_axis,
                             hip_ry_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d hip_ry_intersection = Eigen::Isometry3d::Identity();
  hip_ry_intersection.translation() += hip_ry_intersection_point;

  // now the goal describes the pose of the ankle_intersect in hip_RY_intersect frame
  Eigen::Isometry3d goal = hip_ry_intersection.inverse() * ankle_intersection;
  // compute AnkleRoll
  // Create triangle in y,z dimension consisting of goal position vector and the y and z axis (in hip_RY_intersect frame)
  // Compute atan(goal.y,goal.z), this gives us the angle of the foot as if the goal rotation would be zero.
  double ankle_roll = std::atan2(goal.translation().y(), -goal.translation().z());
  // This is only the angle coming from the goal position. We need to simply add the goal orientation in roll
  // We can compute this joint angles without knowing the HipYaw value, since the position is not influenced by it.
  // Because we use the exact intersection of HipYaw and HipRoll as a basis.
  Eigen::Vector3d goal_rpy = l_sole_goal.rotation().eulerAngles(0, 1, 2);
  ankle_roll += goal_rpy.x();
  goal_state->setJointPositions("LAnkleRoll", &ankle_roll);

  // Compute HipRoll
  // We can compute this similarly as the AnkleRoll. We can also use the triangle method but don't need to add
  // the goal orientation of the foot.
  double hip_roll = std::atan2(goal.translation().y(), -goal.translation().z());
  goal_state->setJointPositions("LHipRoll", &hip_roll);

  // We need to know the position of the HipPitch to compute the rest of the pitch joints.
  // Therefore we need to firstly compute the HipYaw joint, since it influences the position of the HipPitch
  // joint (as their axes do not intersect)
  // First we compute the plane of the leg (the plane in which all pitch joints are).
  // This plane contains the goal position of the ankle_intersect. It can be defined by the normal which is the
  // Y axis in the ankle_intersect frame. //todo nicht sicher ob vorher der roll abgezogen werden muss
  // The hip_RY_intersect is also located on this plane.
  // We determine the intersection of this plane with the xy plane going through hip_ry_intersect
  // Then, we take the angle between this intersection line and the x axis
  // The intersection line can easily be computed with the cross product.
  ankle_pitch_axis =
      goal.rotation() * Eigen::Vector3d(0, 1, 0);  // this is the rotational axis of the ankle pitch, todo get from urdf
  Eigen::Vector3d line = ankle_pitch_axis.cross(Eigen::Vector3d(0, 0, 1));  // this is the normal of the xy plane
  double hip_yaw = std::acos(line.dot(Eigen::Vector3d(1, 0, 0)) / line.norm());
  goal_state->setJointPositions("LHipYaw", &hip_yaw);

  // Represent the goal in the HipPitch frame. Subtract transform from hip_RY_intersect to HipPitch
  goal_state->updateLinkTransforms();
  Eigen::Isometry3d base_link_to_hip_pitch = goal_state->getGlobalLinkTransform("l_upper_leg");
  l_sole_to_l_ankle = goal_state->getGlobalLinkTransform("l_sole").inverse() *
      goal_state->getGlobalLinkTransform("l_ankle");
  Eigen::Isometry3d base_link_to_l_ankle = l_sole_goal * l_sole_to_l_ankle;
  Eigen::Isometry3d hip_pitch_to_l_ankle =  base_link_to_hip_pitch.inverse() * base_link_to_l_ankle;

  // rotation of hip_pitch_to_goal should be zero
  // Now we have a triangle of HipPitch, Knee and AnklePitch which we can treat as a prismatic joint.
  // First we can get the knee angle with the rule of cosine
  // here we get rid of the axis vertical to the plane that should be ignored
  // todo the axis is still hard coded
  Eigen::Isometry3d hip_pitch_to_knee = goal_state->getGlobalLinkTransform("l_upper_leg").inverse() * goal_state->getGlobalLinkTransform("l_lower_leg");
  double upper_leg_length = std::sqrt(std::pow(hip_pitch_to_knee.translation().x(), 2) + std::pow(hip_pitch_to_knee.translation().y(), 2));
  double upper_leg_length_2 = upper_leg_length * upper_leg_length;
  Eigen::Isometry3d knee_to_ankle_pitch = goal_state->getGlobalLinkTransform("l_lower_leg").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  double lower_leg_length = std::sqrt(std::pow(knee_to_ankle_pitch.translation().x(), 2) + std::pow(knee_to_ankle_pitch.translation().z(), 2));
  double lower_leg_length_2 = lower_leg_length * lower_leg_length;
  double hip_to_ankle_length = std::sqrt(std::pow(hip_pitch_to_l_ankle.translation().x(), 2) + std::pow(hip_pitch_to_l_ankle.translation().y(), 2));
  double hip_to_ankle_length_2 = hip_to_ankle_length * hip_to_ankle_length;
  std::cout << upper_leg_length << std::endl << lower_leg_length << std::endl << hip_to_ankle_length << std::endl;
  double knee = std::acos((upper_leg_length_2 + lower_leg_length_2 - hip_to_ankle_length_2) / (2 * upper_leg_length * lower_leg_length));
  // todo actually calculate static offsets
  //knee += 0.241566;  // offset

  // Similarly, we can compute HipPitch and AnklePitch, but we need to add half of the knee angle.
  double hip_pitch = std::acos((upper_leg_length_2 + hip_to_ankle_length_2 - lower_leg_length_2) / (2 * upper_leg_length * hip_to_ankle_length));
  // add pitch of hip_pitch_to_ankle todo not actually hip_pitch_to_l_ankle because this is already rotated?? why??
  double a = std::atan2(hip_pitch_to_l_ankle.translation().x(), -hip_pitch_to_l_ankle.translation().y());  // todo z axis is y axis?
  std::cout << "a " << a << std::endl;
  hip_pitch -= a;
  hip_pitch += 0.026;
  //hip_pitch -= 0.199751;  // offset
  //hip_pitch -= 0.4;
  goal_state->setJointPositions("LHipPitch", &hip_pitch);
  // ankle pitch needs goal pitch
  double ankle_pitch = std::acos((lower_leg_length_2 + hip_to_ankle_length_2 - upper_leg_length_2) / (2 * lower_leg_length * hip_to_ankle_length));
  ankle_pitch += goal_rpy.y();
  //ankle_pitch -= 1.722219;
  Eigen::Isometry3d tf = goal_state->getJointModel("LKnee")->getChildLinkModel()->getJointOriginTransform();
  Eigen::Vector3d knee_axis = dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LKnee"))->getAxis();
  Eigen::Quaterniond knee_zero_pitch_q = getQuaternionTwist(Eigen::Quaterniond(tf.rotation()), knee_axis);
  Eigen::Vector3d euler = Eigen::Matrix3d(knee_zero_pitch_q).eulerAngles(0, 1, 2);
  std::cout << euler;
  double knee_zero_pitch = euler.y();
  //knee -= knee_zero_pitch - M_PI;
  knee += 0.74;  // todo where do you come from?
  goal_state->setJointPositions("LKnee", &knee);

  // subtract hip pitch from ankle pitch
  ankle_pitch += hip_pitch;

  ankle_pitch -= 2.72;
  goal_state->setJointPositions("LAnklePitch", &ankle_pitch);

  return true;
}

Eigen::Quaterniond IK::getQuaternionTwist(const Eigen::Quaterniond& rotation,
                                          const Eigen::Vector3d& direction) {
  Eigen::Vector3d rotation_axis(rotation.x(), rotation.y(), rotation.z());
  Eigen::Vector3d projection = rotation_axis.dot(direction) / (direction.dot(direction)) * direction;
  Eigen::Quaterniond twist(rotation.w(), projection.x(), projection.y(), projection.z());
  twist.normalize();
  return twist;
}

bool IK::findIntersection(const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &v1,
                          const Eigen::Vector3d &p2,
                          const Eigen::Vector3d &v2,
                          Eigen::Vector3d &output) {
  double tolerance = 1e-3;
  // The math behind this is from here:
  // https://web.archive.org/web/20180324134610/https://mathforum.org/library/drmath/view/62814.html
  // Basically, it is the following:
  // Equate the line equations: p1 + a * v1 = p2 + b * v2
  // Reorder: a * v1 = (p2 - p1) + b * v2
  // On both sides, take cross product with v2: a * v1 x v2 = (p2 - p1) x v2
  // Now, we can solve for a by dividing the norms of the vectors.
  // There are two cases where this does not work: v1 x v2 is zero or the vectors are not parallel.
  // In these cases, there is no intersection.

  // calculate v1 x v2
  Eigen::Vector3d v1_v2 = v1.cross(v2);
  if (v1_v2.norm() < tolerance) {
    // v1 x v2 is zero, so there is no intersection
    return false;
  }

  // calculate (p2 - p1) x v2
  Eigen::Vector3d other = (p2 - p1).cross(v2);

  // calculate a by dividing the norm
  double a = other.norm() / v1_v2.norm();

  // now, check if they are parallel
  if ((v1_v2 * a + other).norm() < tolerance) {
    // in this case they are in opposite directions, so just switch the sign of a
    a *= -1;
  } else if ((v1_v2 * a - other).norm() > tolerance) {
    // in this case they are not parallel, so there is no solution
    return false;
  }

  // now calculate the point from the line equation
  output = p1 + a * v1;
  return true;
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<sensor_msgs::JointState>("/config/fake_controller_joint_states", 1);

  wolfgang_ik::IK ik;

  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0.1;
  goal.translation().y() = 0.08;
  goal.translation().z() = -0.3;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state::RobotStatePtr result;
  result.reset(new robot_state::RobotState(kinematic_model));
  ik.solve(goal, result);
  sensor_msgs::JointState joint_state;
  joint_state.name = kinematic_model->getJointModelGroup("LeftLeg")->getJointModelNames();
  result->copyJointGroupPositions("LeftLeg", joint_state.position);
  while (ros::ok()) {
    p.publish(joint_state);
  }
}