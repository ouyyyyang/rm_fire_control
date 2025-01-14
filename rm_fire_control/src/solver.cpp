


namespace rm_auto_aim
{
rm_interfaces::msg::GimbalCmd Solver::Solve(const rm_auto_aim::msh::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_)
{
    // Get current roll, yaw and pitch of gimbal
    try{

    }

    //  predict the the position of target
    Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
    double target_yaw = target.yaw;
    double flying_time = trajectory_compensator_->getFlyingTime(target_position);
    double dt =
        (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
    target_position.x() += dt * target.velocity.x;
    target_position.y() += dt * target.velocity.y;
    target_position.z() += dt * target.velocity.z;
    target_yaw += dt * target.v_yaw;

    // Choose the best armor to shoot
    std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
        target_position, target_yaw, target.radius_1, target.radius_2, target.d_zc, target.d_za, target.armors_num);
    int idx =
        selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);
    auto chosen_armor_position = armor_positions.at(idx);
    if (chosen_armor_position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
    }

    // Calculate yaw, pitch, distance
    double yaw, pitch;
    calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
    double distance = chosen_armor_position.norm();

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;
    gimbal_cmd.distance = distance;
    gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
    switch (state) {
        case TRACKING_ARMOR: {
            if (std::abs(两者的主要区别target.v_yaw) > max_tracking_v_yaw_) {
                overflow_count_++;
            } else {
                overflow_count_ = 0;
            }

            if (overflow_count_ > transfer_thresh_) {
                state = TRACKING_CENTER;
            }

            // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
            if (controller_delay_ != 0) {
                target_position.x() += controller_delay_ * target.velocity.x;
                target_position.y() += controller_delay_ * target.velocity.y;
                target_position.z() += controller_delay_ * target.velocity.z;
                target_yaw += controller_delay_ * target.v_yaw;
                armor_positions = getArmorPositions(target_position,
                                                    target_yaw,
                                                    target.radius_1,
                                                    target.radius_2,
                                                    target.d_zc,
                                                    target.d_za,
                                                    target.armors_num);
                chosen_armor_position = armor_positions.at(idx);
                gimbal_cmd.distance = chosen_armor_position.norm();
                if (chosen_armor_position.norm() < 0.1) {
                    throw std::runtime_error("No valid armor to shoot");
                }
                calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
            }
            break;
        }
        case TRACKING_CENTER: {
            if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
                overflow_count_++;
            } else {
                overflow_count_ = 0;
            }

            if (overflow_count_ > transfer_thresh_) {
                state = TRACKING_ARMOR;
                overflow_count_ = 0;
            }
            gimbal_cmd.fire_advice = true;
            calcYawAndPitch(target_position, rpy_, yaw, pitch);
            break;
    }
  }
   // Compensate angle by angle_offset_map
  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);


  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;  
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double predict_yaw = target_yaw + i * (2 * M_PI / armors_num);

    r = is_current_pair ? r1 : r2;
    target_dz = d_zc + (is_current_pair ? 0 : d_za);
    is_current_pair = !is_current_pair;
    
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(predict_yaw), -r * sin(predict_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  double center_theta = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  double beta = target_yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(center_theta), std::sin(center_theta), 
                  -std::sin(center_theta), std::cos(center_theta);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (center_theta - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

float Solver::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    printf("model %f %f\n", t, z);
    return z;
}

float Solver::pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        printf("iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f\n",
            i + 1, angle_pitch * 180 / PI, z_temp, dz,s);
        if (fabsf(dz) < 0.00001)
        {
            break;
        }
    }
    return angle_pitch;
}

void Solver::autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

    // 线性预测
    float timeDelay = st.bias_time/1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
	int use_1 = 1;
	int i = 0;
    int idx = 0; // 选择的装甲板
    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
    if (st.armor_num == ARMOR_NUM_BALANCE) {
        for (i = 0; i<2; i++) {
            float tmp_yaw = st.tar_yaw + i * PI;
            float r = st.r1;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

        //因为是平衡步兵 只需判断两块装甲板即可
        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }


    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
        for (i = 0; i<3; i++) {
            float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = st.zw;
            tar_position[i].yaw = tmp_yaw;
        }

        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


    } else {

        for (i = 0; i<4; i++) {
            float tmp_yaw = st.tar_yaw + i * PI/2.0;
            float r = use_1 ? st.r1 : st.r2;
            tar_position[i].x = st.xw - r*cos(tmp_yaw);
            tar_position[i].y = st.yw - r*sin(tmp_yaw);
            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
            tar_position[i].yaw = tmp_yaw;
            use_1 = !use_1;
        }

            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板

            //计算距离最近的装甲板
        //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
        //	int idx = 0;
        //	for (i = 1; i<4; i++)
        //	{
        //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
        //		if (temp_dis_diff < dis_diff_min)
        //		{
        //			dis_diff_min = temp_dis_diff;
        //			idx = i;
        //		}
        //	}
        //

            //计算枪管到目标装甲板yaw最小的那个装甲板
        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
        for (i = 1; i<4; i++) {
            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
            if (temp_yaw_diff < yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }

    }

	

    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    //这里符号给错了
    *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
            *aim_z + st.z_bias, st.current_v);
    *yaw = (float)(atan2(*aim_y, *aim_x));

}
}