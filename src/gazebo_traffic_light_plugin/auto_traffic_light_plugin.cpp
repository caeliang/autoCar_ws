/*
 * Automatic Traffic Light Controller Plugin for Gazebo
 * This plugin automatically cycles traffic lights through red, yellow, green states
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Color.hh>

#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <map>

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
      
      // Initialize transport
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_world->Name());
      
      // Publisher for visual messages
      this->visPub = this->node->Advertise<msgs::Visual>("~/visual");
      
      // Setup traffic light groups - mirroring what's in the world file
      // Intersection 1
      groups.push_back(TrafficLightGroup(
        "intersection_1_NS",
        {"stop_light_post_475", "stop_light_post_479"},
        20.0, 3.0, 23.0, 0.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_1_EW",
        {"stop_light_post_482", "stop_light_post_485"},
        20.0, 3.0, 23.0, 23.0  // Offset so EW is red when NS is green
      ));
      
      // Intersection 2
      groups.push_back(TrafficLightGroup(
        "intersection_2_NS",
        {"stop_light_post_476", "stop_light_post_480"},
        20.0, 3.0, 23.0, 5.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_2_EW",
        {"stop_light_post_483"},
        20.0, 3.0, 23.0, 28.0
      ));
      
      // Intersection 3
      groups.push_back(TrafficLightGroup(
        "intersection_3_NS",
        {"stop_light_post_477", "stop_light_post_481"},
        20.0, 3.0, 23.0, 10.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_3_EW",
        {"stop_light_post_486"},
        20.0, 3.0, 23.0, 33.0
      ));
      
      // Intersection 4
      groups.push_back(TrafficLightGroup(
        "intersection_4_NS",
        {"stop_light_post_478"},
        20.0, 3.0, 23.0, 15.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_4_EW",
        {"stop_light_post_484", "stop_light_post_487"},
        20.0, 3.0, 23.0, 38.0
      ));
      
      // Initialize all lights to red
      for (auto& group : groups)
      {
        for (const auto& light : group.lights)
        {
          SetLightColor(light, LightState::RED);
        }
      }
      
      // Connect to world update event
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
        
        // Wrap around cycle
        while (group.elapsedTime >= group.getCycleTime())
        {
          group.elapsedTime -= group.getCycleTime();
        }
        
        // Determine state
        LightState newState;
        if (group.elapsedTime < group.greenTime)
        {
          newState = LightState::GREEN;
        }
        else if (group.elapsedTime < group.greenTime + group.yellowTime)
        {
          newState = LightState::YELLOW;
        }
        else
        {
          newState = LightState::RED;
        }
        
        // Update if state changed
        if (newState != group.currentState)
        {
          group.currentState = newState;
          for (const auto& light : group.lights)
          {
            SetLightColor(light, newState);
          }
          
          // Log state change
          std::string stateStr = (newState == LightState::GREEN) ? "GREEN" :
                                  (newState == LightState::YELLOW) ? "YELLOW" : "RED";
          gzmsg << group.groupId << " -> " << stateStr << std::endl;
        }
      }
    }
    
    void SetLightColor(const std::string& modelName, LightState state)
    {
      // The stop_light_post model contains nested stop_light models
      // Each stop_light has red, yellow, green visuals
      std::vector<std::string> nestedLights = {"right_light", "center_light"};
      std::vector<std::string> colorVisuals = {"red", "yellow", "green"};
      
      // Define emissive colors for each state
      std::map<std::string, ignition::math::Color> emissiveOn, emissiveOff;
      emissiveOn["red"] = ignition::math::Color(1, 0, 0, 1);
      emissiveOn["yellow"] = ignition::math::Color(1, 1, 0, 1);
      emissiveOn["green"] = ignition::math::Color(0, 1, 0, 1);
      
      emissiveOff["red"] = ignition::math::Color(0.1, 0, 0, 1);
      emissiveOff["yellow"] = ignition::math::Color(0.1, 0.1, 0, 1);
      emissiveOff["green"] = ignition::math::Color(0, 0.1, 0, 1);
      
      std::string activeColor;
      switch (state)
      {
        case LightState::RED: activeColor = "red"; break;
        case LightState::YELLOW: activeColor = "yellow"; break;
        case LightState::GREEN: activeColor = "green"; break;
      }
      
      for (const auto& nested : nestedLights)
      {
        for (const auto& colorVis : colorVisuals)
        {
          // Visual name format: modelName::nestedLight::link::visualName
          std::string visualName = modelName + "::" + nested + "::link::" + colorVis;
          
          msgs::Visual visMsg;
          visMsg.set_name(visualName);
          visMsg.set_parent_name(modelName + "::" + nested + "::link");
          
          // Set emissive color
          ignition::math::Color color = (colorVis == activeColor) ? 
                                         emissiveOn[colorVis] : emissiveOff[colorVis];
          
          msgs::Set(visMsg.mutable_material()->mutable_emissive(), color);
          
          this->visPub->Publish(visMsg);
        }
      }
    }
    
  private:
    physics::WorldPtr world;
    transport::NodePtr node;
    transport::PublisherPtr visPub;
    event::ConnectionPtr updateConnection;
    common::Time lastUpdateTime;
    std::vector<TrafficLightGroup> groups;
  };

  GZ_REGISTER_WORLD_PLUGIN(AutoTrafficLightPlugin)
}
