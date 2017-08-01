#include <ros/ros.h>
#include <helelani_common/MissionStart.h>
#include <helelani_common/Mission.h>
#include <std_srvs/Empty.h>

class MissionControlServer
{
    ros::ServiceServer m_startMissionSrv, m_endMissionSrv;
    ros::Publisher m_missionPub;

    helelani_common::Mission m_missionData = {};

    bool _startMission(helelani_common::MissionStart::Request& req,
                       helelani_common::MissionStart::Response& resp)
    {
        m_missionData.elapsed_seconds = req.elapsed_seconds;
        m_missionData.to_rover_delay = req.to_rover_delay;
        m_missionData.from_rover_delay = req.from_rover_delay;
        m_missionData.stereo_camera_type = req.stereo_camera_type;
        m_missionData.mission_active = 1;
    }

    bool _endMission(std_srvs::Empty::Request& req,
                     std_srvs::Empty::Response& resp)
    {
        m_missionData.mission_active = 0;
    }

public:
    explicit MissionControlServer(ros::NodeHandle& n)
    : m_startMissionSrv(n.advertiseService("/helelani/start_mission",
                                           &MissionControlServer::_startMission, this)),
      m_endMissionSrv(n.advertiseService("/helelani/end_mission",
                                         &MissionControlServer::_endMission, this)),
      m_missionPub(n.advertise<helelani_common::Mission>("/helelani/mission", 10, true))
    {}

    void update(const ros::TimerEvent& e)
    {
        if (m_missionData.mission_active)
            m_missionData.elapsed_seconds += 1;
        m_missionPub.publish(m_missionData);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mission_control_server_node");
    ros::NodeHandle n;

    MissionControlServer server(n);

    // Begin update loop
    ros::Timer t = n.createTimer(ros::Duration(1),
                                 &MissionControlServer::update, &server);
    ros::spin();

    return 0;
}
