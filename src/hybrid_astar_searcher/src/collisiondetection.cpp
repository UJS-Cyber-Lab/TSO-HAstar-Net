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
 
 // Input real-world coordinates and yaw angle (x, y, theta).
 bool CollisionDetection::configurationTest(float x, float y, float theta) {
   // Convert real-world coordinates to grid indices.
   int X = std::floor((x - grid->info.origin.position.x) / grid->info.resolution + 0.5);
   int Y = std::floor((y - grid->info.origin.position.y) / grid->info.resolution + 0.5);
 
   // The original theta is in the range [-pi, +pi]. It must be converted to the range [0, 360] degrees.
   if (theta < 0) theta += (2 * M_PI);
   // When casting to int, the result may be -1 due to truncation, causing out-of-bounds issues.
   unsigned int idx = (unsigned int)(theta / Constants::deltaHeadingRad);
   // Match the closest angle index to the current yaw angle.
 
  // int X = (int)x;
  // int Y = (int)y;
  // int iX = (int)((x - (long)x) * Constants::positionResolution); // Calculate the X-direction offset within the cell.
  // iX = iX > 0 ? iX : 0;
  // int iY = (int)((y - (long)y) * Constants::positionResolution); // Calculate the Y-direction offset within the cell.
  // iY = iY > 0 ? iY : 0;
  // We are at the grid origin with a resolution of 0.1, so there is no offset. Offsets only occur at lower resolutions.
  // int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  // unsigned int idx = iT;
  
  // std::cout << "idx:" << idx << std::endl;

   int cX;
   int cY;
 
   // idx is used to represent the coverage grid template for a specific angle.
   for (int i = 0; i < collisionLookup[idx].length; ++i) {
     // Retrieve the coverage grid corresponding to this angle template, with `length` elements. We only use the outline, not the interior.
     // Add the offset (X, Y).
     cX = (X + collisionLookup[idx].pos[i].x);
     cY = (Y + collisionLookup[idx].pos[i].y);
 
     // Ensure the configuration coordinates are within the grid boundaries.
     if (cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
       || grid->data[cY * grid->info.width + cX] == 100 || grid->data[cY * grid->info.width + cX] == -1) {
       // if (grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1) {
       return false;
       // }
       // If there is a value in a small grid of the grid, it indicates that there is an obstacle, and false is returned to indicate that it is not in a free grid.
     }
   }
 
   return true;
   // If no occupied cells are detected, it means there are no obstacles, and the path is clear.
 }
 
 // Input grid coordinates and yaw angle (X, Y, Theta).
 bool CollisionDetection::configurationTest_(int X, int Y, float Theta) {
   // Convert real-world coordinates to grid indices.
   // int X = std::floor((x - grid->info.origin.position.x) / grid->info.resolution + 0.5);
   // int Y = std::floor((y - grid->info.origin.position.y) / grid->info.resolution + 0.5);
 
   // The original theta is in the range [-pi, +pi]. It must be converted to the range [0, 360] degrees.
   if (Theta < 0) Theta += 2 * M_PI;
   int iT = (int)(Theta / Constants::deltaHeadingRad);
   // Match the closest angle index to the current yaw angle.
 
    // int X = (int)x;
    // int Y = (int)y;
    // int iX = (int)((x - (long)x) * Constants::positionResolution); // Calculate the X-direction offset within the cell.
    // iX = iX > 0 ? iX : 0;
    // int iY = (int)((y - (long)y) * Constants::positionResolution); // Calculate the Y-direction offset within the cell.
    // iY = iY > 0 ? iY : 0;
    // We are at the grid origin with a resolution of 0.1, so there is no offset. Offsets only occur at lower resolutions.
    // int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
   int idx = iT;
   int cX;
   int cY;
 
   // idx is used to represent the coverage grid template for a specific angle.
   for (int i = 0; i < collisionLookup[idx].length; ++i) {
     // Retrieve the coverage grid corresponding to this angle template, with `length` elements. We only use the outline, not the interior.
     // Add the offset (X, Y).
     cX = (X + collisionLookup[idx].pos[i].x);
     cY = (Y + collisionLookup[idx].pos[i].y);
 
     // Ensure the configuration coordinates are within the grid boundaries.
     if (cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
       || grid->data[cY * grid->info.width + cX] == 100 || grid->data[cY * grid->info.width + cX] == -1) {
       // if (grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1) {
       return false;
       // }
       // If there is a value in a small grid of the grid, it indicates that there is an obstacle, and false is returned to indicate that it is not in a free grid.
     }
   }
 
   return true;
   // If no occupied cells are detected, it means there are no obstacles, and the path is clear.
 }