#include <iostream>
#include <vector>
#include <queue>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <string>
#include "direction_calculator.h"
#include "waypoint_finder.h"
#include "jps_planner.h"

using namespace std;

// JPS Algoritması Node yapısı
// (jps_planner.h'dan import ediliyor)

// JPS Algoritması kullanacak (A* yerine)

int main(int argc, char** argv) {
    if (argc < 5) {
        cout << "Kullanim: path_planning_main <grid_txt> <waypoint_csv> <arac_x> <arac_y> [hedef_x] [hedef_y]" << endl;
        cout << "Ornek: path_planning_main matrices/road_grid_4wide.txt waypoints/full_road_map.csv 5.5 10.2 50.0 20.0" << endl;
        return 1;
    }

    string grid_file = argv[1];
    string waypoint_file = argv[2];
    double vehicle_x = stod(argv[3]);
    double vehicle_y = stod(argv[4]);

    double target_x, target_y;
    if (argc >= 7) {
        target_x = stod(argv[5]);
        target_y = stod(argv[6]);
    } else {
        // Varsayilan hedef
        target_x = 50.0;
        target_y = 20.0;
    }

    cout << "\n========================================" << endl;
    cout << "     PATH PLANNING SISTEMI" << endl;
    cout << "========================================\n" << endl;

    auto total_start = chrono::high_resolution_clock::now();

    // 1. Waypoint Finder ile en yakin waypoint'i bul
    cout << "[1/3] En yakin waypoint aranıyor..." << endl;
    auto step1_start = chrono::high_resolution_clock::now();
    
    WaypointFinder finder;
    if (!finder.loadWaypoints(waypoint_file)) {
        return -1;
    }

    auto waypoint_result = finder.findNearestWaypoint(vehicle_x, vehicle_y);
    cout << "  Arac Konumu: (" << vehicle_x << ", " << vehicle_y << ")" << endl;
    cout << "  En Yakin Waypoint: [" << waypoint_result.waypoint.index << "]" << endl;
    cout << "  Waypoint Koordinati: (" << fixed << setprecision(3) 
         << waypoint_result.waypoint.x << ", " << waypoint_result.waypoint.y << ")" << endl;
    cout << "  Uzaklik: " << waypoint_result.distance << " birim" << endl;

    auto step1_end = chrono::high_resolution_clock::now();
    auto step1_duration = chrono::duration_cast<chrono::milliseconds>(step1_end - step1_start);
    cout << "  SURE: " << step1_duration.count() << " ms\n" << endl;

    // 2. JPS ile path hesapla
    cout << "[2/3] Jump Point Search algoritması ile rota hesaplanıyor..." << endl;
    auto step2_start = chrono::high_resolution_clock::now();
    
    JPSPlanner planner;
    if (!planner.loadMap(grid_file)) {
        return -1;
    }

    Point start = {waypoint_result.grid_x, waypoint_result.grid_y};
    Point goal = {(int)(target_x), (int)(target_y)};

    cout << "  Baslangic Grid: (" << start.x << ", " << start.y << ")" << endl;
    cout << "  Hedef Grid: (" << goal.x << ", " << goal.y << ")" << endl;

    auto path = planner.findPath(start, goal);

    if (path.empty()) {
        cout << "  Hata: Path bulunamadi!" << endl;
        return -1;
    }

    cout << "  Rota bulundu! " << path.size() << " nokta." << endl;

    auto step2_end = chrono::high_resolution_clock::now();
    auto step2_duration = chrono::duration_cast<chrono::milliseconds>(step2_end - step2_start);
    cout << "  SURE: " << step2_duration.count() << " ms\n" << endl;

    // 3. Yön listesini hesapla
    cout << "[3/3] Yön listesi oluşturuluyor..." << endl;
    auto step3_start = chrono::high_resolution_clock::now();
    
    auto directions = DirectionCalculator::getDirectionsFromPath(path);
    auto direction_strings = DirectionCalculator::directionsToStrings(directions);

    auto step3_end = chrono::high_resolution_clock::now();
    auto step3_duration = chrono::duration_cast<chrono::milliseconds>(step3_end - step3_start);
    cout << "  SURE: " << step3_duration.count() << " ms\n" << endl;

    auto total_end = chrono::high_resolution_clock::now();
    auto total_duration = chrono::duration_cast<chrono::milliseconds>(total_end - total_start);

    // Çıktıyı göster
    cout << "========================================" << endl;
    cout << "     ROTA DETAYLARI" << endl;
    cout << "========================================\n" << endl;

    cout << "Tamamlanacak Adim Sayisi: " << path.size() << endl;
    cout << "Yon Degisikligi Sayisi: " << direction_strings.size() << endl;

    cout << "\n--- ROTANıN DETAYLARI ---\n";
    cout << "Baslangic: Grid[" << path[0].y << "][" << path[0].x << "] (" 
         << path[0].x << ", " << path[0].y << ")" << endl;

    for (size_t i = 0; i < direction_strings.size(); ++i) {
        cout << "Adim " << (i+1) << ": " << direction_strings[i] 
             << " -> Grid[" << path[i+1].y << "][" << path[i+1].x 
             << "] (" << path[i+1].x << ", " << path[i+1].y << ")" << endl;
    }

    cout << "\n--- YON LİSTESİ ---\n";
    cout << "[";
    for (size_t i = 0; i < direction_strings.size(); ++i) {
        cout << direction_strings[i];
        if (i < direction_strings.size() - 1) cout << ", ";
    }
    cout << "]\n" << endl;

    cout << "========================================" << endl;
    cout << "     ROTA HAZIR" << endl;
    cout << "========================================\n" << endl;

    cout << "TOPLAM SURE: " << total_duration.count() << " ms" << endl;
    cout << "  Adim 1 (Waypoint): " << step1_duration.count() << " ms" << endl;
    cout << "  Adim 2 (JPS Path):  " << step2_duration.count() << " ms" << endl;
    cout << "  Adim 3 (Yonler):   " << step3_duration.count() << " ms" << endl;
    cout << "\n========================================\n" << endl;

    return 0;
}
