## Thruster allocator
This package takes a thrust vector and calculates the corresponding thrust required from each individual thruster.
The resulting calculation is published as a vortex_msgs ThrusterForces.

The calculation itself is based on the 'unweighted pseudoinverse-based allocator' as described in _Fossen (2011): Handbook
of Marine Craft Hydrodynamics and Motion Control_ (chapter 12.3.2).
