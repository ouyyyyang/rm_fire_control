


#define YAW_MOTOR_RES_SPEED 1.6f

namespace rm_auto_aim
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:node("firt_control",options)
{
    RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");

    //传入参数
    LargeArmorWidth = this->declare_parameter<float>("LargeArmorWidth", 0.23);  
    SmallArmorWidth = this->declare_parameter<float>("SmallArmorWidth", 0.135);
    k = this->declare_parameter<float>("k", 0.038); //弹丸参数(0.038为小弹丸)

    s_bias = this->declare_parameter<float>("s_bias", );  //s偏移，即枪管口与云台坐标系的xy合向量偏移
    z_bias = this->declare_parameter<float>("z_bias", );  //z偏移，即枪管口与云台坐标系的z向量偏移
    target = this->declare_parameter("",)  
    id_number = this->declare_parameter("",)
    yaw = this->declare_parameter("",)
    r1 = this->declare_parameter("",)
    r2 = this->declare_parameter("",)
    xc = this->declare_parameter("",)  //位置
    yc = this->declare_parameter("",)
    zc = this->declare_parameter("",)
    vx = this->declare_parameter("",)  //速度
    vy = this->declare_parameter("",)
    vz = this->declare_parameter("",)
    vyaw = this->declare_parameter("",)
    dz = this->declare_parameter("",)
    armor_num = this->declare_parameter("",)
    r_timestamp = this->declare_parameter("",)
    first_phase_1 = this->declare_parameter("",)

}

void expected_preview_calc() {
  float armor_x , armor_y , armor_z ;

  VISION_DATA_ASSIGNMENT

  float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;

  // aim_status judge
  if (target == 1) {
    // Prediction
    // predict_time --> shoot delay
    // (now_timestamp - r_timestamp) --> vision calculate using time
    float predict_time_final =
        (float)(now_timestamp - r_timestamp) / 1000 + predict_time;

    xc = xc + predict_time_final * vx;
    yc = yc + predict_time_final * vy;
    zc = zc + predict_time_final * vz;
    float predict_yaw = yaw + predict_time_final * vyaw;
    float center_theta = atan2(yc, xc);

    uint8_t use_1 = 1;
    float diff_angle = 2 * PI / armor_num;

    for (size_t i = 0; i < armor_num; i++) {
      float armor_yaw = predict_yaw + i * diff_angle;
      float yaw_diff = get_delta_ang_pi(armor_yaw, center_theta);
      if (fabsf(yaw_diff) < diff_angle / 2) {
        armor_choose = 1;

        // Only 4 armors has 2 radius and height
        float r = r1;
        armor_z = zc;
        if (armor_num == 4) {
          r = use_1 ? r1 : r2;
          armor_z = use_1 ? zc : (zc + dz);
        }

        // Robot state to armor
        armor_x = xc - r * cos(armor_yaw);
        armor_y = yc - r * sin(armor_yaw);

        // Calculate angle of advance

        float armor_z_next = zc;
        if (armor_num == 4) {
          r = !use_1 ? r1 : r2;
          armor_z_next = !use_1 ? zc : (zc + dz);
        }
        float next_armor_yaw =
            armor_yaw -
            sign(vyaw) * diff_angle;  // TODO: use future span to calculate
                                      // target delta angle
        float armor_x_next = xc - r * cos(next_armor_yaw);
        float armor_y_next = yc - r * sin(next_armor_yaw);

        float yaw_motor_delta =
            get_delta_ang_pi(atan2(armor_y, armor_x),
                             atan2(armor_y_next, armor_x_next));
        float angle_of_advance =
            fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(vyaw) / 2;

        float est_yaw;
        if (sign(vyaw) * yaw_diff < diff_angle / 2 - angle_of_advance ||
            angle_of_advance > diff_angle / 4) {
          est_x = armor_x;
          est_y = armor_y;
          est_z = armor_z;
          est_yaw = armor_yaw;
        } else {
          est_x = armor_x_next;
          est_y = armor_y_next;
          est_z = armor_z_next;
          est_yaw = next_armor_yaw;
        }

        // Calculate fire control params
        float armor_w;
        if (armor_num == 2 || id_num == 1)
          armor_w = LARGE_ARMOR_WIDTH;
        else
          armor_w = SmallArmorWidth;
        float ax = est_x - 0.5f * armor_w * sin(est_yaw);
        float ay = est_y + 0.5f * armor_w * cos(est_yaw);
        float bx = est_x + 0.5f * armor_w * sin(est_yaw);
        float by = est_y - 0.5f * armor_w * cos(est_yaw);
        float angle_a = atan2(ay, ax);
        float angle_b = atan2(by, bx);
        float angle_c = atan2(est_y, est_x);
        allow_fire_ang_max = angle_c - angle_b;
        allow_fire_ang_min = angle_c - angle_a;

        break;
      } else {
        armor_choose = 0;
      }
      use_1 = !use_1;
    }
  }
  fire_ctrl(allow_fire_ang_max, allow_fire_ang_min, target);
}
}