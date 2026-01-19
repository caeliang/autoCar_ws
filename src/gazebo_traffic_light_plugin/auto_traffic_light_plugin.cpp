/*
 * Automatic Traffic Light Controller Plugin for Gazebo
 * Simple version using Gazebo transport for visual updates
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Color.hh>

#include <vector>
#include <string>

namespace gazebo
{
  enum class LightState { RED, YELLOW, GREEN };

  struct TrafficLightGroup
  {
    std::string groupId;
    std::vector<std::string> lights;
    LightState currentState;
    double greenTime;
    double yellowTime;
    double redTime;
    double elapsedTime;
    
    TrafficLightGroup(const std::string& id, 
                      const std::vector<std::string>& lightNames,
                      double green = 20.0, 
                      double yellow = 3.0,
                      double red = 23.0,
                      double startOffset = 0.0)
      : groupId(id), lights(lightNames), currentState(LightState::RED),
        greenTime(green), yellowTime(yellow), redTime(red), elapsedTime(startOffset)
    {}
    
    double getCycleTime() const { return greenTime + yellowTime + redTime; }
  };

  class AutoTrafficLightPlugin : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      this->world = _world;
      
      // Initialize Gazebo transport
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_world->Name());
      this->visPub = this->node->Advertise<msgs::Visual>("~/visual");
      
      // Wait for publisher to connect
      this->visPub->WaitForConnection();
      
      // Read log_messages parameter (optional, default: false)
      bool logMessages = false;
      if (_sdf && _sdf->HasElement("log_messages"))
      {
        logMessages = _sdf->Get<bool>("log_messages");
      }
      this->logMessages = logMessages;
      
      // Detect world and setup appropriate traffic light groups
      std::string worldName = _world->Name();
      
      if (worldName.find("compact_city") != std::string::npos)
      {
        // compact_city.world.backup - 12 lights (3 intersections with 4 lights each)
        // All lights work together in sync
        groups.push_back(TrafficLightGroup("compact_city_all", 
          {"stop_light_post_1", "stop_light_post_2", "stop_light_post_3", "stop_light_post_4",
           "stop_light_post_5", "stop_light_post_6", "stop_light_post_7", "stop_light_post_8",
           "stop_light_post_9", "stop_light_post_10", "stop_light_post_11", "stop_light_post_12"}, 
          20.0, 3.0, 23.0, 0.0));
        
        if (this->logMessages)
          gzmsg << "Loaded compact_city traffic lights configuration (12 lights)" << std::endl;
      }
      else
      {
        // simple_city_copy.world - Multiple intersections with alternating lights
        groups.push_back(TrafficLightGroup("intersection_1_NS", 
          {"stop_light_post_475", "stop_light_post_479"}, 20.0, 3.0, 23.0, 0.0));
        groups.push_back(TrafficLightGroup("intersection_1_EW", 
          {"stop_light_post_482", "stop_light_post_485"}, 20.0, 3.0, 23.0, 23.0));
        
        groups.push_back(TrafficLightGroup("intersection_2_NS", 
          {"stop_light_post_476", "stop_light_post_480"}, 20.0, 3.0, 23.0, 5.0));
        groups.push_back(TrafficLightGroup("intersection_2_EW", 
          {"stop_light_post_483"}, 20.0, 3.0, 23.0, 28.0));
        
        groups.push_back(TrafficLightGroup("intersection_3_NS", 
          {"stop_light_post_477", "stop_light_post_481"}, 20.0, 3.0, 23.0, 10.0));
        groups.push_back(TrafficLightGroup("intersection_3_EW", 
          {"stop_light_post_486"}, 20.0, 3.0, 23.0, 33.0));
        
        groups.push_back(TrafficLightGroup("intersection_4_NS", 
          {"stop_light_post_478"}, 20.0, 3.0, 23.0, 15.0));
        groups.push_back(TrafficLightGroup("intersection_4_EW", 
          {"stop_light_post_484", "stop_light_post_487"}, 20.0, 3.0, 23.0, 38.0));
        
        if (this->logMessages)
          gzmsg << "Loaded simple_city traffic lights configuration" << std::endl;
      }
      
      // Initialize all lights to RED
      for (auto& group : groups)
      {
        for (const auto& lightName : group.lights)
        {
          SetLightColor(lightName, LightState::RED);
        }
      }
      
      // Connect to world update
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AutoTrafficLightPlugin::OnUpdate, this, std::placeholders::_1));
      
      this->lastUpdateTime = this->world->SimTime();
      
      gzmsg << "AutoTrafficLightPlugin loaded successfully!" << std::endl;
    }
    
    void OnUpdate(const common::UpdateInfo& info)
    {
      common::Time currentTime = this->world->SimTime();
      double dt = (currentTime - this->lastUpdateTime).Double();
      this->lastUpdateTime = currentTime;
      
      for (auto& group : groups)
      {
        group.elapsedTime += dt;
        while (group.elapsedTime >= group.getCycleTime())
          group.elapsedTime -= group.getCycleTime();
        
        LightState newState;
        if (group.elapsedTime < group.greenTime)
          newState = LightState::GREEN;
        else if (group.elapsedTime < group.greenTime + group.yellowTime)
          newState = LightState::YELLOW;
        else
          newState = LightState::RED;
        
        // Update visuals when state changes
        if (newState != group.currentState)
        {
          group.currentState = newState;
          
          if (this->logMessages)
          {
            std::string stateStr = (newState == LightState::GREEN) ? "GREEN" :
                                   (newState == LightState::YELLOW) ? "YELLOW" : "RED";
            gzmsg << group.groupId << " changed to " << stateStr << std::endl;
          }
          
          for (const auto& lightName : group.lights)
          {
            SetLightColor(lightName, newState);
          }
        }
      }
    }
    
    void SetLightColor(const std::string& modelName, LightState state)
    {
      // Set emissive colors for each light based on state
      ignition::math::Color redOn(1, 0, 0, 1);
      ignition::math::Color redOff(0.2, 0, 0, 1);
      ignition::math::Color yellowOn(1, 1, 0, 1);
      ignition::math::Color yellowOff(0.2, 0.2, 0, 1);
      ignition::math::Color greenOn(0, 1, 0, 1);
      ignition::math::Color greenOff(0, 0.2, 0, 1);
      
      // Update both right_light and center_light
      std::vector<std::string> lightTypes = {"right_light", "center_light"};
      
      for (const auto& lightType : lightTypes)
      {
        // Update red light
        UpdateVisual(modelName + "::" + lightType + "::link::red", 
                     (state == LightState::RED) ? redOn : redOff);
        
        // Update yellow light
        UpdateVisual(modelName + "::" + lightType + "::link::yellow", 
                     (state == LightState::YELLOW) ? yellowOn : yellowOff);
        
        // Update green light
        UpdateVisual(modelName + "::" + lightType + "::link::green", 
                     (state == LightState::GREEN) ? greenOn : greenOff);
      }
    }
    
    void UpdateVisual(const std::string& visualName, const ignition::math::Color& emissive)
    {
      msgs::Visual visualMsg;
      visualMsg.set_name(visualName);
      visualMsg.set_parent_name(visualName.substr(0, visualName.rfind("::")));
      
      // Set both emissive and ambient for better visibility
      msgs::Set(visualMsg.mutable_material()->mutable_emissive(), emissive);
      msgs::Set(visualMsg.mutable_material()->mutable_ambient(), emissive);
      msgs::Set(visualMsg.mutable_material()->mutable_diffuse(), emissive);
      
      this->visPub->Publish(visualMsg);
    }
    
  private:
    physics::WorldPtr world;
    transport::NodePtr node;
    transport::PublisherPtr visPub;
    event::ConnectionPtr updateConnection;
    common::Time lastUpdateTime;
    std::vector<TrafficLightGroup> groups;
    bool logMessages;
  };

  GZ_REGISTER_WORLD_PLUGIN(AutoTrafficLightPlugin)
}
