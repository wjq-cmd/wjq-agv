// 说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示全部打开
#include <lib_library.h>
#include "complete_mission.h"
#include <std_msgs/Int32.h>
#include <string>
#include "ctrl_trans_msgs/ctrl_mode.h"
// 声明来自complete_mission.h的外部变量，用于重置窗口识别标志位
extern float object_recognize_track_last_time_position_x;
extern float object_recognize_track_last_time_position_y;
extern bool  object_recognize_track_flag;
/*--------------------------------------------------
   全局变量
--------------------------------------------------*/
int  mission_num   = 0;
int  tmp_marker_id = 1;
int  target_qr_id  = 1;  // 当前任务的目标二维码ID

/* Python 节点 qr_land_flag.py 发布的对准完成标志 */
bool qr_land_done_flag = false;
int current_qr_id = 0;

/* 防止重复发布目标ID */
int last_published_target_qr_id = -1;

/* 地面任务控制模式 */
ctrl_trans_msgs::ctrl_mode ctrl_mode_msg;

/* 时间记录函数实现 */
bool lib_time_record_func(float time_duration, ros::Time time_now)
{
    if (!lib_time_record_start_flag)
    {
        lib_mission_success_time_record = time_now;
        lib_time_record_start_flag = true;
        return false;
    }
    
    if (time_now - lib_mission_success_time_record > ros::Duration(time_duration))
    {
        lib_time_record_start_flag = false;
        return true;
    }
    
    return false;
}

/*--------------------------------------------------
   回调：Python 节点二维码就绪标志
--------------------------------------------------*/
void qr_land_cb(const std_msgs::Bool::ConstPtr &msg)
{
    qr_land_done_flag = msg->data;
}

/*--------------------------------------------------
   回调：Python 节点二维码ID
--------------------------------------------------*/
void qr_id_cb(const std_msgs::Int32::ConstPtr &msg)
{
    current_qr_id = msg->data;
}

/*--------------------------------------------------
   回调：地面任务控制模式
--------------------------------------------------*/
void ctrl_mode_cb(const ctrl_trans_msgs::ctrl_mode::ConstPtr& msg)
{
    ctrl_mode_msg = *msg;
}

int main(int argc, char **argv)
{
    /* 防止中文输出乱码 */
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "complete_mission");
    ros::NodeHandle nh;

    /*--------------- 订阅 ---------------*/
    ros::Subscriber state_sub      = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub  = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(
        "object_position", 100, object_pos_cb);
    ros::Subscriber cmd_vel_sub    = nh.subscribe(
        "/cmd_vel", 10, cmd_to_px4);

    /* 订阅 Python 节点输出 */
    ros::Subscriber qr_done_sub = nh.subscribe<std_msgs::Bool>(
        "qr_land_done", 10, qr_land_cb);
    ros::Subscriber qr_id_sub = nh.subscribe<std_msgs::Int32>(
        "qr_code_id", 10, qr_id_cb);
    
    /* 订阅地面任务控制模式 */
    ros::Subscriber ctrl_mode_sub = nh.subscribe<ctrl_trans_msgs::ctrl_mode>(
        "/ctrl_mode", 10, ctrl_mode_cb);

    /*--------------- 发布 ---------------*/
    ros::Publisher mavros_setpoint_pos_pub =
        nh.advertise<mavros_msgs::PositionTarget>(
            "/mavros/setpoint_raw/local", 100);
    
    /* 发布目标二维码ID给二维码检测节点 */
    ros::Publisher target_qr_id_pub = 
        nh.advertise<std_msgs::Int32>("/target_qr_id", 10);

    /*--------------- 服务 ---------------*/
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>(
            "mavros/cmd/arming");
    ros::ServiceClient set_mode_client =
        nh.serviceClient<mavros_msgs::SetMode>(
            "mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client =
        nh.serviceClient<mavros_msgs::CommandLong>(
            "mavros/cmd/command");

    ros::Rate rate(20);

    /* 等待与飞控连接 */
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /* 起飞前固定 setpoint */
    setpoint_raw.type_mask        = 64 + 128 + 256 + 512;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x       = 0;
    setpoint_raw.position.y       = 0;
    setpoint_raw.position.z       = ALTITUDE;
    setpoint_raw.yaw              = 0;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    /* 设置模式 / 解锁 */
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    /* 舵机复位 */
    lib_pwm_control(0, 0);
    ctrl_pwm_client.call(lib_ctrl_pwm);

    /******************** 等待地面任务完成 ********************/
    ctrl_mode_msg.ctrl_mode = 0;
    while(ros::ok())
    {
        if(ctrl_mode_msg.ctrl_mode == 1) 
        {
            break;
        }
        printf("ctrl_mode = %d\n", ctrl_mode_msg.ctrl_mode);
        ROS_INFO("等待地面任务完成中...");
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("地面任务完成，开始无人机任务");

    /******************** 起飞阶段 ********************/
    while (ros::ok())
    {
        /* 切换 OFFBOARD */
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        /* 解锁 */
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        /* 到达高度 ALTITUDE 后停 5 秒稳定悬停再进入任务 */
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.1)
        {
            if (ros::Time::now() - last_request > ros::Duration(1.0))
            {
                ROS_INFO("到达目标高度，开始稳定悬停");
                mission_num = 1;
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    /******************** 主任务状态机 ********************/
    bool       tmp_time_record_start_flag       = false;
    ros::Time  tmp_mission_success_time_record;
    float      tmp_x = 0, tmp_y = 0;
    int        tmp_mission = 1;
    bool       object_data_cleared = false;  // 添加标志位，确保清空旧的识别数据
    
    // 窗口搜索超时保护变量
    ros::Time  window_search_start_time;      // 窗口搜索开始时间

    while (ros::ok())
    {
        if (tmp_mission != mission_num)
        {
            tmp_mission = mission_num;
            printf("mission_num = %d\r\n", mission_num);
        }

        switch (mission_num)
        {
        /*---------------- 任务 1：稳定悬停阶段 ----------------*/
        case 1:
            if (mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2))
            {
                ROS_INFO("稳定悬停完成，准备开始窗口识别");
                object_data_cleared = false;  // 重置清空标志
                window_search_start_time = ros::Time::now();  // 记录搜索开始时间
                mission_num = 2;
            }
            break;

        case 2: /* 窗口识别 */
            if (object_recognize_track("wujiaqi", 0, 1, ALTITUDE, 20, 0.1))
                mission_num = 3;
            break;

        case 3: /* 穿过窗口 */
            if (target_through(2.7, 0.0, ALTITUDE, 0))
                mission_num = 4;
            break;

        /*---------------- 第 1 个任务点 ----------------*/
        case 4:
            if (mission_pos_cruise(3.1, -0.1, ALTITUDE, 0, 0.2))
                mission_num = 41;
            break;
        case 41:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))
                mission_num = 5;
            break;
        case 42: /* 二维码对准，由 Python 节点判定 
            target_qr_id = 3;  // 第1个任务点的目标ID
            /* 只有目标ID变化时才发布，避免重复发布 */
            if (target_qr_id != last_published_target_qr_id) {
                std_msgs::Int32 target_id_msg;
                target_id_msg.data = target_qr_id;
                target_qr_id_pub.publish(target_id_msg);
                last_published_target_qr_id = target_qr_id;
                ROS_INFO("发布新的目标二维码ID: %d", target_qr_id);
            }
            if (current_qr_id == target_qr_id && qr_land_done_flag)
            {
                if (!tmp_time_record_start_flag)
                {
                    tmp_time_record_start_flag = true;
                    lib_pwm_control(100, 100);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    ROS_INFO("识别到目标二维码ID: %d，开启激光笔", target_qr_id);
                    tmp_mission_success_time_record = ros::Time::now();
                }
                if (ros::Time::now() - tmp_mission_success_time_record >
                    ros::Duration(3.0))
                {
                    ROS_INFO("关闭激光笔");
                    lib_pwm_control(0, 0);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    tmp_time_record_start_flag = false;
                    qr_land_done_flag          = false;
                    mission_num                = 5;
                }
            }
            else if (lib_time_record_func(5.0, ros::Time::now()))
            {
                mission_num = 5;
                ROS_INFO("二维码超时或ID不匹配，当前ID: %d, 目标ID: %d", current_qr_id, target_qr_id);
            }
            break;

        /*---------------- 第 2 个任务点 ----------------*/
        case 5:
            if (mission_pos_cruise(3.1, 1.35, ALTITUDE, 0,0.2))//2.7稍微前进
                mission_num = 51;
            break;
        case 51:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))//误差似乎可以适当调小
                mission_num = 6;
            break;
        case 52:
            target_qr_id = 4;  // 第2个任务点的目标ID
            /* 只有目标ID变化时才发布，避免重复发布 */
            if (target_qr_id != last_published_target_qr_id) {
                std_msgs::Int32 target_id_msg;
                target_id_msg.data = target_qr_id;
                target_qr_id_pub.publish(target_id_msg);
                last_published_target_qr_id = target_qr_id;
                ROS_INFO("发布新的目标二维码ID: %d", target_qr_id);
            }
            if (current_qr_id == target_qr_id && qr_land_done_flag)
            {
                if (!tmp_time_record_start_flag)
                {
                    tmp_time_record_start_flag = true;
                    lib_pwm_control(100, 100);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    ROS_INFO("识别到目标二维码ID: %d，开启激光笔", target_qr_id);
                    tmp_mission_success_time_record = ros::Time::now();
                }
                if (ros::Time::now() - tmp_mission_success_time_record >
                    ros::Duration(1.0))
                {
                    ROS_INFO("关闭激光笔");
                    lib_pwm_control(0, 0);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    tmp_time_record_start_flag = false;
                    qr_land_done_flag          = false;
                    mission_num                = 6;
                }
            }
            else if (lib_time_record_func(5.0, ros::Time::now()))
            {
                mission_num = 6;
                ROS_INFO("二维码超时或ID不匹配，当前ID: %d, 目标ID: %d", current_qr_id, target_qr_id);
            }
            break;

        /*---------------- 第 3 个任务点 ----------------*/
        case 6:
            if (mission_pos_cruise(3.1, 2.7, ALTITUDE, 0,0.2))//2.4->2.3需要有明显飞行动作，取消明显飞行动作，角度可以考虑在第二个任务点转向
                mission_num = 61;
            break;
        case 61:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 0, 0.2))
                mission_num = 7;
            break;
        case 62:
            target_qr_id = 2;  // 第3个任务点的目标ID
            /* 只有目标ID变化时才发布，避免重复发布 */
            if (target_qr_id != last_published_target_qr_id) {
                std_msgs::Int32 target_id_msg;
                target_id_msg.data = target_qr_id;
                target_qr_id_pub.publish(target_id_msg);
                last_published_target_qr_id = target_qr_id;
                ROS_INFO("发布新的目标二维码ID: %d", target_qr_id);
            }
            if (current_qr_id == target_qr_id && qr_land_done_flag)
            {
                if (!tmp_time_record_start_flag)
                {
                    tmp_time_record_start_flag = true;
                    lib_pwm_control(100, 100);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    ROS_INFO("识别到目标二维码ID: %d，开启激光笔", target_qr_id);
                    tmp_mission_success_time_record = ros::Time::now();
                }
                if (ros::Time::now() - tmp_mission_success_time_record >
                    ros::Duration(1.0))
                {
                    ROS_INFO("关闭激光笔");
                    lib_pwm_control(0, 0);
					ctrl_pwm_client.call(lib_ctrl_pwm);
                    tmp_time_record_start_flag = false;
                    qr_land_done_flag          = false;
                    mission_num                = 7;
                }
            }
            else if (lib_time_record_func(2.0, ros::Time::now()))
            {
                mission_num = 7;
                ROS_INFO("二维码超时或ID不匹配，当前ID: %d, 目标ID: %d", current_qr_id, target_qr_id);
            }
            break;

        /*---------------- 第 4 个任务点 + 出窗 ----------------*/
        case 7:
            if (mission_pos_cruise(3.1, 2.7, ALTITUDE, 0,0.2))//准备穿越第二个窗口，3.0往后调整3.3，这里调用moa避障函数防撞，不对没开地图没用
                mission_num = 71;
            break;
        case 71:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 3.14, 0.2))//转向，同时误差回调防止原地卡自己悬停
                mission_num = 711;
            break;
        
        case 711: /* 第二次穿窗前稳定悬停 */
            if (lib_time_record_func(2.0, ros::Time::now()))  // 稳定悬停1秒，悬停时间太长容易漂
            {
                ROS_INFO("第二次穿窗前稳定悬停完成，开始窗口识别");
                // 重置窗口识别标志位，确保第二次识别能重新初始化
                object_recognize_track_flag = false;
                ROS_INFO("重置窗口识别标志位，准备第二次窗口识别");
                mission_num = 72;
            }
            break;
            
        case 72:
            if (object_recognize_track("wujiaqi", 3.14, -1, ALTITUDE, 20, 0.1))
            {
                // 重置穿窗标志位，确保第二次穿窗能重新初始化位置
                extern bool target_through_init_position_flag;
                target_through_init_position_flag = false;
                ROS_INFO("重置穿窗标志位，准备第二次穿窗");
                mission_num = 73;//在第二个窗口识别到就直接降落
            }
            break;

        case 73: /* 第二次穿窗 */
            if (target_through(-2.8, 0, ALTITUDE, 3.14))
            {
                mission_num = 81;//穿窗后直接降落
                ROS_INFO("穿窗成功");
            }
            break;

        /*---------------- 终点 1 ----------------*/
        case 8:
            if (mission_pos_cruise(0.5, 1.7, ALTITUDE, 3.14,0.2))//需要在终点降落
            {
                mission_pos_cruise_flag = false;
                mission_num            = 81;
            }
            break;
        case 81:
            if (current_position_cruise(0.0, 0.0, ALTITUDE, 3.14, 0.2))
                mission_num = 10;//到达终点直接降落
            break;
        case 82:
            target_qr_id = 3;  // 第4个任务点的目标ID
            /* 只有目标ID变化时才发布，避免重复发布 */
            if (target_qr_id != last_published_target_qr_id) {
                std_msgs::Int32 target_id_msg;
                target_id_msg.data = target_qr_id;
                target_qr_id_pub.publish(target_id_msg);
                last_published_target_qr_id = target_qr_id;
                ROS_INFO("发布新的目标二维码ID: %d", target_qr_id);
            }
            if (current_qr_id == target_qr_id && qr_land_done_flag)
            {
                if (lib_time_record_func(0.1, ros::Time::now()))
                {
                    tmp_x              = local_pos.pose.pose.position.x;
                    tmp_y              = local_pos.pose.pose.position.y;
                    qr_land_done_flag  = false;
                    mission_num        = 9;
                    ROS_INFO("识别到目标二维码ID: %d，准备降落", target_qr_id);
                }
            }
            else if (lib_time_record_func(5.0, ros::Time::now()))
            {
                mission_num = 9;
                ROS_INFO("二维码超时或ID不匹配，当前ID: %d, 目标ID: %d", current_qr_id, target_qr_id);
            }
            break;

        case 9: /* 再下降 0.2 m */
            if (mission_pos_cruise(tmp_x, tmp_y, -0.2, 3.14, 0.2) &&
                lib_time_record_func(1.0, ros::Time::now()))//tmp_x,tmp_y继承自识别到二维码的位置
                mission_num = 10;
            break;

        case 10: /* AUTO.LAND */
            ROS_INFO("AUTO.LAND");
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            mission_num = 10;
            break;
        }

        /* 发布 setpoint */
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}