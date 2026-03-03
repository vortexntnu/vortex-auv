colcon build --packages-up-to pool_exploration --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1


Må endre på testfilene som er laget, bare fått ut noe med chat

Utvide til venstre hjørne også





Skal det sjekkes med filter???









Får ikke inn parametrisering av linjene som rho, theta likevel (?)
 //static LineSegment rhoThetaToSegment(double rho, double theta, float length); //Bestem om den er static/vanlig medlem eller utenfor senere
    

/*
LineSegment PoolExplorationMap::rhoThetaToSegment(double rho, double theta, float length) {
    // Finn punkt på linjen nærmest origo
    double x0 = rho * cos(theta);
    double y0 = rho * sin(theta);

    // Retningsvektor for linjen
    double dx = -sin(theta);
    double dy = cos(theta);

    LineSegment seg;
    seg.p1 = Eigen::Vector2f(x0 + (length)/2*dx, y0 + (length)/2*dy);
    seg.p2 = Eigen::Vector2f(x0 - (length)/2*dx, y0 - (length)/2*dy);
    return seg;
}

*/