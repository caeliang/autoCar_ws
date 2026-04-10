#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include "direction_calculator.h"
#include "waypoint_finder.h"
#include "jps_planner.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 8) {
        cerr << "Kullanim: path_planning_export <map.txt> <waypoints.csv> <vehicle_x> <vehicle_y> <goal_x> <goal_y> <output.csv>" << endl;
        return 1;
    }

    string map_file = argv[1];
    string waypoint_file = argv[2];
    double vehicle_x = stod(argv[3]);
    double vehicle_y = stod(argv[4]);
    double goal_x = stod(argv[5]);
    double goal_y = stod(argv[6]);
    string output_file = argv[7];

    // Waypoint'leri yükle
    cout << "[1/3] Waypoint'ler yükleniyor..." << endl;
    WaypointFinder wf;
    if (!wf.loadWaypoints(waypoint_file)) {
        cerr << "Hata: Waypoint dosyasi acilamadi!" << endl;
        return 1;
    }

    auto nearest = wf.findNearestWaypoint(vehicle_x, vehicle_y);
    Point start_grid = nearest.gridIndex;
    
    cout << "  Baslangic: (" << start_grid.x << ", " << start_grid.y << ")" << endl;

    // JPS ile path hesapla
    cout << "[2/3] JPS ile path hesaplanıyor..." << endl;
    JPSPlanner planner;
    if (!planner.loadMap(map_file)) {
        cerr << "Hata: Harita dosyasi acilamadi!" << endl;
        return 1;
    }

    Point goal_grid = {(int)goal_x, (int)goal_y};
    auto t1 = chrono::high_resolution_clock::now();
    vector<Point> path = planner.findPath(start_grid, goal_grid);
    auto t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

    if (path.empty()) {
        cerr << "Hata: Path bulunamadi!" << endl;
        return 1;
    }

    cout << "  Path bulundu: " << path.size() << " nokta, " << duration << " ms" << endl;

    // Yönleri hesapla
    cout << "[3/3] Yonler hesaplanıyor..." << endl;
    auto directions = DirectionCalculator::getDirectionsFromPath(path);

    // CSV'ye kaydet
    cout << "CSV'ye yaziliyor: " << output_file << endl;
    ofstream outfile(output_file);
    outfile << "step,x,y,direction\n";
    
    for (size_t i = 0; i < path.size(); ++i) {
        string dir = (i < directions.size()) ? 
            DirectionCalculator::directionToString(directions[i]) : "N/A";
        outfile << i << "," << path[i].x << "," << path[i].y << "," << dir << "\n";
    }
    outfile.close();

    cout << "Tamamlandı!" << endl;
    cout << "SURE: " << duration << " ms" << endl;

    return 0;
}
