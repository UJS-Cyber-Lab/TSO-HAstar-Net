/**
 * @brief Collision detection function set
 */
 #include "hybrid_astar_searcher/collisiondetection.h"
 #include <iostream>
 #include <chrono>
 using namespace planning;
 
 CollisionDetection::CollisionDetection() {
   this->grid = nullptr;
   // collisionLookup is the generated coverage grid template.
   auto dp_map_start = std::chrono::high_resolution_clock::now();
   Lookup::collisionLookup(collisionLookup);
   auto dp_map_end = std::chrono::high_resolution_clock::now();
   std::chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
   std::cout << "Occupancy grid template generation time: " << dp_map_use_time.count() * 1000 << "ms" << std::endl;
   std::cout << "CollisionDetection Init!" << std::endl;
 }
 
 bool CollisionDetection::configurationTest(float x, float y, float theta) {
   int X = std::floor((x - grid->info.origin.position.x) / grid->info.resolution + 0.5);
   int Y = std::floor((y - grid->info.origin.position.y) / grid->info.resolution + 0.5);
 
   if (theta < 0) theta += (2 * M_PI);
   unsigned int idx = (unsigned int)(theta / Constants::deltaHeadingRad);
   // Match the closest angle index to the current yaw angle.

   int cX;
   int cY;
 
   for (int i = 0; i < collisionLookup[idx].length; ++i) {
     cX = (X + collisionLookup[idx].pos[i].x);
     cY = (Y + collisionLookup[idx].pos[i].y);
 
     // Ensure the configuration coordinates are within the grid boundaries.
     if (cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
       || grid->data[cY * grid->info.width + cX] == 100 || grid->data[cY * grid->info.width + cX] == -1) {
       return false;
     }
   }
 
   return true;
 }
 
 bool CollisionDetection::configurationTest_(int X, int Y, float Theta) {
   if (Theta < 0) Theta += 2 * M_PI;
   int iT = (int)(Theta / Constants::deltaHeadingRad);
 
   int idx = iT;
   int cX;
   int cY;
 
   for (int i = 0; i < collisionLookup[idx].length; ++i) {
     cX = (X + collisionLookup[idx].pos[i].x);
     cY = (Y + collisionLookup[idx].pos[i].y);
 
     // Ensure the configuration coordinates are within the grid boundaries.
     if (cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
       || grid->data[cY * grid->info.width + cX] == 100 || grid->data[cY * grid->info.width + cX] == -1) {
       return false;
     }
   }
 
   return true;
 }