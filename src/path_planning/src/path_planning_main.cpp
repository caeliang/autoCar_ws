#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;

// Global variables to store received data
atomic<bool> received_waypoint(false);
atomic<bool> received_grid_index(false);
geometry_msgs::msg::Point latest_waypoint;
geometry_msgs::msg::Point latest_grid_index;

/**
 * ROS Node to listen to waypoint_tracker and car_index_finder outputs
 */
class PathPlanningMainNode : public rclcpp::Node {
public:
    PathPlanningMainNode() : Node("path_planning_main_node") {
        // Subscribe to waypoint tracker output
        wp_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/prius/nearest_waypoint", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                latest_waypoint = *msg;
                received_waypoint = true;
            }
        );
        
        // Subscribe to car index finder output
        grid_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/prius/car_matrix_index", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                latest_grid_index = *msg;
                received_grid_index = true;
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Path Planning Main Node initialized. Waiting for tracker and finder...");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr wp_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr grid_sub_;
};

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Kullanim: path_planning_main <hedef_grid_x> <hedef_grid_y>" << endl;
        cout << "Ornek: path_planning_main 45 55" << endl;
        return 1;
    }

    int goal_x = stoi(argv[1]);
    int goal_y = stoi(argv[2]);

    cout << "\n========================================" << endl;
    cout << "     PATH PLANNING (A* + ROS)" << endl;
    cout << "========================================\n" << endl;

    auto total_start = chrono::high_resolution_clock::now();

    // Step 1: Start tracker and finder processes
    cout << "[1/4] ROS node'ları (tracker ve finder) arka planda başlatılıyor..." << endl;
    
    rclcpp::init(argc, argv);
    auto main_node = make_shared<PathPlanningMainNode>();
    
    // Start waypoint tracker in background
    system("ros2 run path_planning waypoint_tracker &");
    
    // Start car index finder in background
    system("ros2 run path_planning car_index_finder &");
    
    this_thread::sleep_for(chrono::milliseconds(1000)); // Give them time to start
    cout << "✓ Node'lar başlatıldı\n" << endl;

    // Step 2: Generate route using A* algorithm (via generate_route.py)
    cout << "[2/4] A* algoritması ile rota hesaplanıyor..." << endl;
    auto step2_start = chrono::high_resolution_clock::now();
    
    string cmd = "python3 /home/ranim/autoCar_ws/src/path_planning/scripts/generate_route.py " + 
                 to_string(goal_x) + " " + to_string(goal_y) + " >/dev/null 2>&1";
    int result = system(cmd.c_str());
    
    if (result != 0) {
        cout << "✗ Rota hesaplaması başarısız!" << endl;
        return -1;
    }
    cout << "✓ Rota oluşturuldu: planned_route.csv\n" << endl;

    auto step2_end = chrono::high_resolution_clock::now();
    auto step2_duration = chrono::duration_cast<chrono::milliseconds>(step2_end - step2_start);

    // Step 3: Wait for waypoint tracker and car index finder to provide data
    cout << "[3/4] Aracın o anki konumu ve matris indeksi alınıyor..." << endl;
    auto step3_start = chrono::high_resolution_clock::now();
    
    int max_wait = 30; // seconds
    int waited = 0;
    
    while (!received_waypoint.load() || !received_grid_index.load()) {
        rclcpp::spin_some(main_node);
        this_thread::sleep_for(chrono::milliseconds(100));
        waited += 100;
        
        if (waited > max_wait) {
            cout << "✗ Zaman aşımı! Tracker veya finder veri göndermiyor." << endl;
            return -1;
        }
    }
    
    auto step3_end = chrono::high_resolution_clock::now();
    auto step3_duration = chrono::duration_cast<chrono::milliseconds>(step3_end - step3_start);
    
    cout << "✓ Veriler alındı\n" << endl;

    // Step 4: Display results
    cout << "[4/4] Sonuçlar gösteriliyor..." << endl << endl;

    cout << "========================================" << endl;
    cout << "     ROTA PLANLAMA SONUCU" << endl;
    cout << "========================================\n" << endl;

    cout << "Hedef Matris İndeksi: [X: " << goal_x << ", Y: " << goal_y << "]" << endl << endl;

    cout << "Araç O Anki Durumu:" << endl;
    cout << "  Dünya Koordinatları: (" << fixed << setprecision(2) 
         << latest_waypoint.x << ", " << latest_waypoint.y << ")" << endl;
    cout << "  Matris İndeksi: [X: " << (int)latest_grid_index.x 
         << ", Y: " << (int)latest_grid_index.y << "]" << endl << endl;

    cout << "Rota Bilgisi:" << endl;
    cout << "  A* Planlama Süresi: " << step2_duration.count() << " ms" << endl;
    cout << "  Konum Algılama Süresi: " << step3_duration.count() << " ms" << endl << endl;

    auto total_end = chrono::high_resolution_clock::now();
    auto total_duration = chrono::duration_cast<chrono::milliseconds>(total_end - total_start);

    cout << "========================================" << endl;
    cout << "TOPLAM IŞLEM SÜRESİ: " << total_duration.count() << " ms" << endl;
    cout << "========================================\n" << endl;

    rclcpp::shutdown();
    return 0;
}
