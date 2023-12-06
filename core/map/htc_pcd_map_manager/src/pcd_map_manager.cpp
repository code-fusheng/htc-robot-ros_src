#include "pcd_map_manager.h"

pcd_map_manager::pcd_map_manager(ros::NodeHandle &nh, ros::NodeHandle &private_nh){};

pcd_map_manager::~pcd_map_manager() {};

void pcd_map_manager::run()
{
    // init params
    init_params();

    // def publisher
    points_map_pub = nh.advertise<sensor_msgs::PointCloud2>("points_map", 1, true);
    static_map_pub = nh.advertise<sensor_msgs::PointCloud2>("static_map", 1, true);

    // def subscriber
    current_pose_sub = nh.subscribe("/current_pose", 10, &pcd_map_manager::callback_current_pose, this);
    inittial_pose_sub = nh.subscribe("/initial_pose", 10, &pcd_map_manager::callback_init_pose, this);

	default_AreaList = read_arealist(dir_dynamic_map + "/" + AREALIST_FILENAME);
	publish_sparse_map();

    ros::spin();

};

void pcd_map_manager::init_params() 
{

    private_nh.param<double>("margin", MARGIN, 100.0);
    private_nh.param<double>("update_interval", UPDATE_INTERVAL, 1000.0);

    dir_static_map = "";    
    dir_dynamic_map = "";

    private_nh.param<std::string>("dir_static_map", dir_static_map, "");
    private_nh.param<std::string>("dir_dynamic_map", dir_dynamic_map, "");

};

void pcd_map_manager::callback_current_pose(const geometry_msgs::PoseStamped &msg) 
{
    if (dir_dynamic_map == "") return;
    static ros::Time last_update_time = ros::Time::now();
    if ((msg.header.stamp - last_update_time).toSec() * 1000 < UPDATE_INTERVAL)
		return;
	publish_pcd(create_pcd(msg.pose.position));
	last_update_time = msg.header.stamp;
};

void pcd_map_manager::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
	if (dir_dynamic_map == "") {
		ROS_WARN("[PCD Map Manager] Received /initial_pose, but dynamic map dir is empty.");
		return;
	}
	geometry_msgs::PoseStamped cur_pose;
	cur_pose.header = msg.header;
	cur_pose.pose = msg.pose.pose;
	publish_pcd(create_pcd(msg.pose.pose.position));
};

sensor_msgs::PointCloud2 pcd_map_manager::create_pcd(const geometry_msgs::Point &p) 
{
	AreaList wanted_AreaList = create_wantedAreaList(p);
	AreaList tmp_AreaList;

	bool cache_update = false;
	for (Area &area : wanted_AreaList)
	{
		int index = is_in_cachedAreaList(area);
		if (index == -1)
		{ // io
			Area in_area(area);
			pcl::io::loadPCDFile(dir_dynamic_map + "/" + area.filename, in_area.points);
			tmp_AreaList.push_back(in_area);
			cache_update = true;
		}
		else
		{
			Area in_area(cached_AreaList[index]);
			tmp_AreaList.push_back(in_area);
		}
	}
	if (cache_update)
	{
		cached_AreaList.clear();
		cached_AreaList = tmp_AreaList;
	}
	else
	{
		sensor_msgs::PointCloud2 msg_pcd;
		msg_pcd.width = 0;
		return msg_pcd;
	}

	sensor_msgs::PointCloud2 msg_pcd;
	for (const Area &area : cached_AreaList)
	{
		if (msg_pcd.width == 0)
			msg_pcd = area.points;
		else
		{
			msg_pcd.width += area.points.width;
			msg_pcd.row_step += area.points.row_step;
			msg_pcd.data.insert(msg_pcd.data.end(), area.points.data.begin(), area.points.data.end());
		}
	}
    ROS_INFO("[PCD Map Manager] Created pcd map size = %d", msg_pcd.width);
	return msg_pcd;
};

pcd_map_manager::AreaList pcd_map_manager::create_wantedAreaList(const geometry_msgs::Point &p)
{
    AreaList ret;
	for (const Area &area : default_AreaList)
	{
		if (is_in_area(p.x, p.y, area, MARGIN))
		{
			Area in_area(area);
			ret.push_back(in_area);
		}
	}
	return ret;
};

bool pcd_map_manager::is_in_area(double x, double y, const Area &area, double m) 
{
    return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
};

int pcd_map_manager::is_in_cachedAreaList(Area wanted_area) 
{
	int length = cached_AreaList.size();
	for (int i = 0; i < length; i++)
	{
		if (wanted_area.filename == cached_AreaList[i].filename)
		{
			return i;
		}
	}
	return -1;
};

void pcd_map_manager::publish_pcd(sensor_msgs::PointCloud2 pcd)
{
	if (pcd.width != 0)
	{
		pcd.header.frame_id = "map";
		points_map_pub.publish(pcd);
		ROS_INFO("[pcd_map_manager] Publish map with &d points", pcd.width);
		return;
	}
	ROS_INFO("[pcd_map_manager] No points to publish");
};

pcd_map_manager::AreaList pcd_map_manager::read_arealist(const std::string &path)
{
	std::ifstream ifs(path.c_str());
	std::string line;
	AreaList ret;
	while (std::getline(ifs, line))
	{
		std::istringstream iss(line);
		std::string col;
		std::vector<std::string> cols;
		while (std::getline(iss, col, ','))
			cols.push_back(col);
		Area tmp;
		tmp.filename = cols[0];
		tmp.x_min = std::stod(cols[1]);
		tmp.y_min = std::stod(cols[2]);
		tmp.z_min = std::stod(cols[3]);
		tmp.x_max = std::stod(cols[4]);
		tmp.y_max = std::stod(cols[5]);
		tmp.z_max = std::stod(cols[6]);
		ret.push_back(tmp);
	}
	return ret;
};

bool pcd_map_manager::publish_sparse_map() {
	ros::Duration(3.0).sleep();
	sensor_msgs::PointCloud2::Ptr msg_globalmap(new sensor_msgs::PointCloud2);
	int res = pcl::io::loadPCDFile(dir_static_map+"/static.pcd", *msg_globalmap);
	if (res != 0)
	{
		ROS_WARN("[pcd map manager] Cann't load map");
		return false;
	}
	msg_globalmap->header.frame_id = "map";
	static_map_pub.publish(*msg_globalmap);
	return true;
}
