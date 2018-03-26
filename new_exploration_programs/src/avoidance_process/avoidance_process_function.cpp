#include <new_exploration_programs/avoidance_process.h>
//#include <new_exploration_programs/basic_process.h>

void AvoidanceProcess::obstacle_avoidance(void)
{
  float plus_ave;
  float minus_ave;
  float sum = 0;
  float range_threshold = 1.0;

  std::vector<float> ranges;
  float angle_min;
  float angle_max;
  float angle_increment;

  float omega;
  bp.get_omega(&omega);

  bp.get_scan(&ranges, &angle_min, &angle_max, &angle_increment);

  bp.approx(ranges);

  //minus側の平均
  for(int i=0;i<ranges.size()/2;i++){
    if(!isnan(ranges[i])){
      sum += ranges[i];
    }
  }
  minus_ave = sum/(ranges.size()/2);

  sum = 0;

  //plus側
  for(int i=ranges.size()/2;i<ranges.size();i++){
    if(!isnan(ranges[i])){
      sum += ranges[i];
    }
  }
  plus_ave = sum/(ranges.size()/2);

  //平均を比較

  std::cout << "plus_ave:" << plus_ave << std::endl;
  std::cout << "minus_ave:" << minus_ave << std::endl;

  if(plus_ave < range_threshold && minus_ave < range_threshold){
    bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3,0.0);
  }
  else{
    if(omega > 0 && plus_ave > range_threshold){
      avoidance_sign = 1.0;
      bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3);
    }

    else if(omega < 0 && minus_ave > range_threshold){
      avoidance_sign = -1.0;
      bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3);
    }
    else if(plus_ave > minus_ave){
      std::cout << "plusに回転\n" << std::endl;
      avoidance_sign = 1.0;
      bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3);
    }
    else if(plus_ave < minus_ave){
      std::cout << "minusに回転\n" << std::endl;
      avoidance_sign = -1.0;
      bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3);
    }
    else{
      std::cout << "無理です\n" << std::endl;
      bp.vel_curve_VFH(avoidance_sign*angle_max/6,0.3,0.0);
    }
  }
}

void AvoidanceProcess::bumper_avoidance(void)
{
  bool bumper_hit;
  int which_bumper;
  float vel_x;
  float vel_z;

  const float rotate_time = 1.5;

  float rotate_vel;
  float pre_theta;

  ros::Time set_time;
  ros::Duration rotate(rotate_time);

  bp.get_bumper(&bumper_hit,&which_bumper);
  bp.get_rotatevel(&rotate_vel);
  bp.get_pretheta(&pre_theta);

  if(bumper_hit)
  {
    //後ろに下がる処理
    go_back();

    vel_x = 0;

    if(which_bumper == 0)
    {
      vel_z = -rotate_vel;
      bp.set_vel(vel_x,vel_z);
    }
    else if(which_bumper == 2)
    {
      vel_z = rotate_vel;
      bp.set_vel(vel_x,vel_z);
    }
    else
    {
      if(pre_theta > 0)
      {
        vel_z = -rotate_vel;
        bp.set_vel(vel_x,vel_z);
      }
      else
      {
        vel_z = rotate_vel;
        bp.set_vel(vel_x,vel_z);
      }
    }
    set_time = ros::Time::now();
    while(ros::Time::now()-set_time < rotate)
    {
	bp.pub_vel();
    }
  }
}

void AvoidanceProcess::go_back(void)
{
    const float back_vel = -0.2;
    const float back_time = 0.5;
    ros::Time set_time;
    ros::Duration back(back_time);

    float vel_x = back_vel;
    float vel_z = 0;
    bp.set_vel(vel_x,vel_z);

    set_time = ros::Time::now();

    while(ros::Time::now()-set_time < back){
			bp.pub_vel();
		}
}
