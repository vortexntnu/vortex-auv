#include <iostream>
#include <pool_exploration/pool_exploration.hpp>
#include <Eigen/Core>

int main() {
    // Lag et lite kart
    vortex::pool_exploration::PoolExplorationMap map(5.0, 5.0, 0.1, "map");

    // Sjekk at grid er initialisert
    const auto& grid = map.grid();
    std::cout << "Grid width: " << grid.info.width << ", height: " << grid.info.height << std::endl;
    std::cout << "Første celle: " << grid.data[0] << std::endl;

    // Sett en celle
    map.setGridCell(0, 0, 100);
    std::cout << "Etter setting: " << grid.data[0] << std::endl;

    return 0;
}
