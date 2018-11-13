#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_control/psm_controller.h>
#include <cwru_davinci_playfile/playfile_format_cartesian.h>

int main(int argc, char **argv)
{
  // Locate our file.
  std::vector<std::vector<double> > data_pre = std::vector<std::vector<double> >();
  if (argc == 2)
  {
    cartesian_format::read_file(std::string(argv[1]), data_pre);
  }
  else if (argc == 3)
  {
    cartesian_format::read_file(std::string(argv[1]), std::string(argv[2]), data_pre);
  }
  else
  {
    ROS_ERROR("Missing file or package location. Aborting.");
    return 0;
  }

  // Use kinematics to convert portal space into joint space.
  std::vector<std::vector<double> > data = std::vector<std::vector<double> >(data_pre.size());

  davinci_kinematics::Inverse kin = davinci_kinematics::Inverse();

  // Added to use the new kinematics with yaml files. 
  // RN 20180722
  kin.resetDhOffsetsMaps();
  kin.loadDHyamlfiles("psm1_dh","psm1_dh");
  kin.loadDHyamlfiles("psm2_dh","psm2_dh");
  // kin.loadDHyamlfiles("psm_generic","psm_generic");



  for (int i = 0; i < data_pre.size(); i++)
  {
    data[i] = std::vector<double>(15);

    // Copy over the time and gripper open angles, since kinematics does not affect these.
    data[i][14] = data_pre[i][20];
    data[i][13] = data_pre[i][19];
    data[i][6]  = data_pre[i][9];

    // PSM1
    // TODO(tes77) Move pre-processing into kinematics? It's a bit repetitive...
    Eigen::Vector3d tip_origin(data_pre[i][0], data_pre[i][1], data_pre[i][2]);
    Eigen::Vector3d x_vec(data_pre[i][3], data_pre[i][4], data_pre[i][5]);
    Eigen::Vector3d z_vec(data_pre[i][6], data_pre[i][7], data_pre[i][8]);
    Eigen::Vector3d y_vec = z_vec.cross(x_vec);

    Eigen::Affine3d des_gripper_affine;
    Eigen::Matrix3d R;

    R.col(0) = x_vec;
    R.col(1) = y_vec;
    R.col(2) = z_vec;
    des_gripper_affine.linear() = R;
    des_gripper_affine.translation() = tip_origin;

    // if (kin.ik_solve_refined(des_gripper_affine) <= 0)
    if (kin.ik_solve_refined(des_gripper_affine, "psm1_dh") <= 0)
		// if (kin.ik_solve_frozen_refined(des_gripper_affine, "psm1_dh") <= 0)
    // if (kin.ik_solve_frozen_refined(des_gripper_affine, "psm_generic") <= 0)
    {
      ROS_ERROR("Line %d does not have a kinematic solution for PSM1!", i);
      // TODO(tes77) Abort pending resolution of kinematics issue.
      return 0;
    }

    // davinci_kinematics::Vectorq7x1 solution = kin.get_soln_refined();
    davinci_kinematics::Vectorq7x1 solution = kin.get_soln_refined("psm1_dh"); 
		// davinci_kinematics::Vectorq7x1 solution = kin.get_soln_frozon_ik_refined("psm1_dh");
    // davinci_kinematics::Vectorq7x1 solution = kin.get_soln_frozon_ik_refined("psm_generic"); 
    for (int j = 0; j < 6; j++)
    {
      data[i][j] = solution[j];
    }




    // PSM2
    tip_origin = Eigen::Vector3d(data_pre[i][10], data_pre[i][11], data_pre[i][12]);
    x_vec = Eigen::Vector3d(data_pre[i][13], data_pre[i][14], data_pre[i][15]);
    z_vec = Eigen::Vector3d(data_pre[i][16], data_pre[i][17], data_pre[i][18]);
    y_vec = z_vec.cross(x_vec);

    R.col(0) = x_vec;
    R.col(1) = y_vec;
    R.col(2) = z_vec;
    des_gripper_affine.linear() = R;
    des_gripper_affine.translation() = tip_origin;

    // if (kin.ik_solve_refined(des_gripper_affine) <= 0)
    if (kin.ik_solve_refined(des_gripper_affine, "psm2_dh") <= 0)
    // if (kin.ik_solve_frozen_refined(des_gripper_affine, "psm2_dh") <= 0)
    {
      ROS_ERROR("Line %d does not have a kinematic solution for PSM2!", i);
      return 0;
    }

    // solution = kin.get_soln_refined();
    solution = kin.get_soln_refined("psm2_dh"); 
    // solution = kin.get_soln_frozon_ik_refined("psm2_dh");
    for (int j = 0; j < 7; j++)
    {
      data[i][j+7] = solution[j];
    }
  }


  return 0;
}
