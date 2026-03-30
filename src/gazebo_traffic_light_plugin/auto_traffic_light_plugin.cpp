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
      
      // Setup traffic light groups - mirroring what's in the world file (compact_city.world)
      // Intersection 1
      groups.push_back(TrafficLightGroup(
        "intersection_1_NS",
        {"stop_light_post_1", "stop_light_post_5"},
        20.0, 3.0, 23.0, 0.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_1_EW",
        {"stop_light_post_10", "stop_light_post_6"},
        20.0, 3.0, 23.0, 23.0  // Offset so EW is red when NS is green
      ));
      
      // Intersection 2
      groups.push_back(TrafficLightGroup(
        "intersection_2_NS",
        {"stop_light_post_11", "stop_light_post_12"},
        20.0, 3.0, 23.0, 5.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_2_EW",
        {"stop_light_post_2"},
        20.0, 3.0, 23.0, 28.0
      ));
      
      // Intersection 3
      groups.push_back(TrafficLightGroup(
        "intersection_3_NS",
        {"stop_light_post_3", "stop_light_post_4"},
        20.0, 3.0, 23.0, 10.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_3_EW",
        {"stop_light_post_7"},
        20.0, 3.0, 23.0, 33.0
      ));
      
      // Intersection 4
      groups.push_back(TrafficLightGroup(
        "intersection_4_NS",
        {"stop_light_post_8"},
        20.0, 3.0, 23.0, 15.0
      ));
      
      groups.push_back(TrafficLightGroup(
        "intersection_4_EW",
        {"stop_light_post_9"},
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
          /*
          std::string stateStr = (newState == LightState::GREEN) ? "GREEN" :
                                  (newState == LightState::YELLOW) ? "YELLOW" : "RED";
          gzmsg << group.groupId << " -> " << stateStr << std::endl;
          */
        }
      }
    }
    
    void SetLightColor(const std::string& modelName, LightState state)
    {
        std::string activeColor;
        switch (state)
        {
            case LightState::RED:    activeColor = "red";    break;
            case LightState::YELLOW: activeColor = "yellow"; break;
            case LightState::GREEN:  activeColor = "green";  break;
        }

        // Define emissive colors for each state
        std::map<std::string, ignition::math::Color> colorOn, colorOff;

        colorOn["red"]    = ignition::math::Color(1.0, 0.0, 0.0, 1);
        colorOn["yellow"] = ignition::math::Color(1.0, 0.8, 0.0, 1);
        colorOn["green"]  = ignition::math::Color(0.0, 1.0, 0.0, 1);

        // Dim (mat) versions of the same colors
        colorOff["red"]    = ignition::math::Color(0.25, 0.0, 0.0, 1);
        colorOff["yellow"] = ignition::math::Color(0.25, 0.2, 0.0, 1);
        colorOff["green"]  = ignition::math::Color(0.0, 0.25, 0.0, 1);
        
        std::vector<std::string> nestedLights = {"right_light", "center_light"};
        std::vector<std::string> colorVisuals = {"red", "yellow", "green"};

        for (const auto& nested : nestedLights)
        {
            for (const auto& colorVis : colorVisuals)
            {
                // Try both visual name formats to be sure
                std::vector<std::string> visualNames = {
                    modelName + "::" + nested + "::link::" + colorVis,
                    modelName + "::" + nested + "::" + colorVis
                };

                for (const auto& visualName : visualNames)
                {
                    msgs::Visual visMsg;
                    visMsg.set_name(visualName);
                    visMsg.set_parent_name(modelName);
                    
                    ignition::math::Color color = (colorVis == activeColor) ? 
                                                   colorOn[colorVis] : colorOff[colorVis];
                    
                    msgs::Material* mat = visMsg.mutable_material();
                    
                    // Do NOT set empty script name, it causes "Empty string used when setting a required parameter" error
                    // Instead, we will just set the color properties directly which should override the script
                    
                    // Set all color components to ensure visibility
                    msgs::Set(mat->mutable_ambient(), color);
                    msgs::Set(mat->mutable_diffuse(), color);
                    msgs::Set(mat->mutable_specular(), color);
                    msgs::Set(mat->mutable_emissive(), color);
                    
                    this->visPub->Publish(visMsg);
                }
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
