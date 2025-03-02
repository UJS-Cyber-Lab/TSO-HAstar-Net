#include "hybrid_astar_searcher/dynamicvoronoi.h"

#include <math.h>
#include <iostream>


DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
}


// Initialize the data array, which is the data structure for each cell on the map
void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX; // Map dimensions
  sizeY = _sizeY;
  // If the data array already exists, free its memory space first.
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  // Allocate a new data array
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];
  // If initGridMap is true (we are false), perform the following operations:
  if (initGridMap) {
    // If the gridMap array already exists (we do), free its memory space first.
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    // Allocate a new gridMap array and initialize all its elements to 0.
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }

  // Set the initial attribute values for each cell to infinity (dist), maximum integer (sqdist), invalid obstacle data (obstX and obstY), free state (voronoi), not queued (queueing), and no need for raising (needsRaise).
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  // Initialize
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;
  // Assign values

  // We are false for initGridMap
  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
    // Clear grid state
  }
}


// Process all obstacle grids, paying special attention to boundary obstacle grids
void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);
  // Initialize the data array
  // Traverse the entire grid map
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {

      // For each obstacle cell (true), perform the following operations:
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        // Check if the given coordinates (x and y) are obstacles. Actually, this indicates that the obstacle cell has not been initialized yet.
        if (!isOccupied(x,y,c)) {
          // Not initialized:
          bool isSurrounded = true;
          // Initialize as true
          // Check if the cell at the given coordinates (x and y) is surrounded by its 8 neighboring cells
          // Use a nested loop to traverse the 8 neighboring cells around the current cell (excluding itself)
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;
              // For each neighboring cell, if the cell is not occupied (i.e., gridMap[nx][ny] is false), set isSurrounded to false and break out of the inner loop.
              if (!gridMap[nx][ny]) {
                // Indicates that the current cell has at least one neighboring cell that is not occupied, making it a boundary cell
                isSurrounded = false;
                break;
              }
            }
          }
          // If isSurrounded is true, it means all 8 neighboring cells of the current cell are obstacle cells
          if (isSurrounded) {
            // obstX and obstY: The coordinates of the nearest obstacle cell, which is itself
            c.obstX = x;
            c.obstY = y;
            // sqdist: The squared distance from the cell to the nearest obstacle cell, which is itself
            c.sqdist = 0;
            // dist: The distance from the cell to the nearest obstacle cell
            c.dist=0;
            c.voronoi=occupied; // Fully occupied obstacle cell
            c.queueing = fwProcessed; // Processed
            data[x][y] = c;
            // Boundary cell: Mark it as an obstacle cell. Ignore inner cells
          } else setObstacle(x,y);
          // We only care about boundary obstacles
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}



void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  // Already set as an obstacle
  
  addList.push_back(INTPOINT(x,y));
  // Add to addList, which stores all updated boundary obstacles
  c.obstX = x;
  // Itself
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(const std::vector<INTPOINT>& points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();
  lastObstacles.reserve(points.size());

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
}



void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);
  // Add or remove obstacle cells and update obstacle information

  // Calculate the distance from each cell to the nearest obstacle and update the Voronoi diagram
  while (!open.empty()) {
    INTPOINT p = open.pop();
    // Pop a coordinate p with the smallest sqdist value from the open queue. Initially, open contains boundary obstacles, and later it propagates to free cells.
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    // RAISE
    if (c.needsRaise) {
      // Traverse the 8 neighboring cells around the current cell
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny]; // Neighboring cell
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            // If the neighboring cell nc is not an obstacle, add it to the open queue and set its queueing property to fwQueued
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              // Add to open based on sqdist
              nc.queueing = fwQueued;
              // Enqueue
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              // If updateRealDist is true, set the dist property of the neighboring cell nc to infinity, indicating that the distance from this cell to the obstacle is infinite
              if (updateRealDist) nc.dist = INFINITY;
              // Initialize
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
              // If the neighboring cell nc is an obstacle, add it to the open queue and set its queueing property to fwQueued.
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                // Add to open
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      // Set the queueing property of the current cell c to bwProcessed, indicating that the cell has been processed
      data[x][y] = c;
    }

    // If the data cell c belongs to an obstacle
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {
      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;
      // Obstacle

      // Traverse the 8 neighboring cells around the current cell
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;	
            // Calculate the squared distance from each neighboring cell to the obstacle
            bool overwrite =  (newSqDistance < nc.sqdist);
            // If the squared distance from the neighboring cell nc to the obstacle equals the original distance saved in nc, and nc's original obstX is not invalid, and the obstacle has not been reinserted:
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            // If the squared distance from the neighboring cell nc to the obstacle is less than the original distance saved in nc (i.e., nc.sqdist):
            if (overwrite) {
              // Add the coordinates of the neighboring cell nc to the open queue
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              // If updateRealDist is true, calculate the real distance from the neighboring cell nc to the obstacle and update the dist property of nc.
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              // Set the sqdist property of the neighboring cell nc to the new distance value.
              nc.sqdist = newSqDistance;
              // Update the obstX and obstY properties of the neighboring cell nc to the coordinates of the obstacle.
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              // Call the checkVoro function to check the Voronoi boundary between the current cell c and the neighboring cell nc, and update the Voronoi diagram.
              // Cells equidistant to multiple obstacles will enter here
              checkVoro(x,y,nx,ny,c,nc);
            }
            // Write the updated neighboring cell nc back to the corresponding position in the data array.
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}


// Get the grid distance to the nearest obstacle cell
float DynamicVoronoi::getDistance(int x, int y) const {
  if ((x > 0) && (x < sizeX) && (y > 0) && (y < sizeY)) return data[x][y].dist;
  else return -INFINITY;
}


// Check if the given coordinates (x, y) belong to the Voronoi diagram
bool DynamicVoronoi::isVoronoi(int x, int y) const {
  dataCell c = data[x][y];

  return (c.voronoi == free || c.voronoi == voronoiKeep);
  // Valid Voronoi node
}


// Add or remove obstacle cells and update obstacle information
void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  // Traverse the addList (which stores all updated boundary obstacle cells) and process newly added obstacles. For each coordinate p, get its x and y coordinates, and retrieve the corresponding data cell c from data[x][y].
  for (unsigned int i = 0; i < addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    // If the queueing property of the data cell c is not fwQueued (indicating that the cell is not marked as an obstacle)
    if (c.queueing != fwQueued) {
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      // Itself
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      // Enqueue
      c.voronoi = occupied;
      // Indicates that the cell belongs to an obstacle
      data[x][y] = c;
      open.push(0, INTPOINT(x, y));
      // Transfer addList to the open queue
    }
  }

  // REMOVE OLD OBSTACLES
  // Traverse the removeList and process removed obstacles
  for (unsigned int i = 0; i < removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x, y, c) continue;
    // Obstacle was removed and reinserted, indicating that the cell is still marked as an obstacle
    open.push(0, INTPOINT(x, y));
    // Add to open, all changes must be added, marked?
    if (updateRealDist) c.dist = INFINITY;
    // Infinity, initialized?
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


// Check the Voronoi boundary between the current cell c and the neighboring cell nc, and update the Voronoi diagram
/* If there is an obstacle between the current cell c and the neighboring cell nc, then there is a Voronoi boundary between them.
   To calculate the Voronoi boundary, the function first computes the distance from the current cell c to the obstacle corresponding to the neighboring cell nc (i.e., the stability_xy variable), and the distance from the neighboring cell nc to the obstacle corresponding to the current cell c (i.e., the stability_nxy variable).
   Then, based on the relationship between the two distances, it determines which cell should be added to the Voronoi diagram. */
void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist > 1 || nc.sqdist > 1) && nc.obstX != invalidObstData) {
    if (abs(c.obstX - nc.obstX) > 1 || abs(c.obstY - nc.obstY) > 1) {
      // Compute distance from x,y to the obstacle of nx,ny
      int dxy_x = x - nc.obstX;
      int dxy_y = y - nc.obstY;
      int sqdxy = dxy_x * dxy_x + dxy_y * dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist < 0) return;

      // Compute distance from nx,ny to the obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x * dnxy_x + dnxy_y * dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist < 0) return;

      // Which cell is added to the Voronoi diagram?
      /* If the distance from the current cell c to the obstacle corresponding to the neighboring cell nc (stability_xy) is less than or equal to the distance from the neighboring cell nc to the obstacle corresponding to the current cell c (stability_nxy),
         and the squared distance from the current cell c to the obstacle (c.sqdist) is greater than 2, then add the current cell c to the Voronoi diagram.
         To do this, set the voronoi property of the current cell c to free, indicating that the cell belongs to the Voronoi diagram, and set the voronoi property of its eight neighboring cells to free as well for further processing. */
      if (stability_xy <= stability_nxy && c.sqdist > 2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          // Originally occupied
          reviveVoroNeighbors(x, y);
          // If the current cell c or the neighboring cell nc is added to the Voronoi diagram, add their coordinates to the pruneQueue for processing during Voronoi diagram pruning
          pruneQueue.push(INTPOINT(x, y));
        }
      }
      /* If the distance from the neighboring cell nc to the obstacle corresponding to the current cell c (stability_nxy) is less than or equal to the distance from the current cell c to the obstacle corresponding to the neighboring cell nc (stability_xy),
         and the squared distance from the neighboring cell nc to the obstacle (nc.sqdist) is greater than 2, then add the neighboring cell nc to the Voronoi diagram. To do this, set the voronoi property of the neighboring cell nc to free, and set the voronoi property of its eight neighboring cells to free as well. */
      if (stability_nxy <= stability_xy && nc.sqdist > 2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx, ny);
          pruneQueue.push(INTPOINT(nx, ny));
        }
      }
    }
  }
}


// Remove the qualified neighboring cells around the specified coordinates (x, y) from the Voronoi diagram and add them to the pruning queue for further processing.
void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  // Traverse the neighboring cells around it
  for (int dx = -1; dx <= 1; dx++) {
    int nx = x + dx;
    if (nx <= 0 || nx >= sizeX - 1) continue;
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      int ny = y + dy;
      if (ny <= 0 || ny >= sizeY - 1) continue;
      dataCell nc = data[nx][ny];
      // Check if the distance value nc.sqdist of the neighboring cell is not equal to the maximum integer value INT_MAX, it does not need to be raised (needsRaise is false), and its voronoi property is voronoiKeep or voronoiPrune.
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        // If the above conditions are met, set the voronoi property of the neighboring cell to free
        nc.voronoi = free;
        // Then write the updated neighboring cell data back to the data array and add its coordinates to the pruneQueue for further processing during Voronoi diagram pruning.
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx, ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) const {
  dataCell c = data[x][y];
  return (c.obstX == x && c.obstY == y);
}


// Check if the given coordinates (x and y) are obstacles
bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) {
  return (c.obstX == x && c.obstY == y);
}


// Visualize the Voronoi diagram by writing it to a file
void DynamicVoronoi::visualize(const char *filename) {
  // Write pgm files
  std::cout << "Visualizing" << std::endl;
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "Could not open 'result.pgm' for writing!\n";
    return;
  }
  // P6 format is a binary format where each pixel is represented by three bytes corresponding to the red, green, and blue channels.
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  // Traverse each pixel in the Voronoi diagram and write the corresponding color values to the file based on its Voronoi state and distance value.
  for (int y = sizeY - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX; x++) {
      unsigned char c = 0;
      if (isVoronoi(x, y)) {
        // The current pixel belongs to the Voronoi diagram
        fputc(255, F);
        // Red
        fputc(0, F);
        fputc(0, F);
      } else if (data[x][y].sqdist == 0) {
        // Source point
        fputc(0, F);
        // Black
        fputc(0, F);
        fputc(0, F);
      } else {
        float f = 80 + (data[x][y].dist * 5);
        if (f > 255) f = 255;
        if (f < 0) f = 0;
        c = (unsigned char)f;
        fputc(c, F);
        // Gradient grayscale values represent distance information
        fputc(c, F);
        fputc(c, F);
      }
    }
  }
  fclose(F);
}


// Pruning
// Set unnecessary nodes to pruning state (voronoiPrune), keep state (voronoiKeep), or retry state (voronoiRetry)
void DynamicVoronoi::prune() {
  // Filler
  // Traverse the points in the pruneQueue
  while (!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    // Take a point p from the pruneQueue
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi == occupied) continue;
    if (data[x][y].voronoi == freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p); // Add to the open priority queue

    /* tl t tr
       l c r
       bl b br */

    // Top-right, bottom-right, top-left, bottom-left
    dataCell tr, tl, br, bl;
    tr = data[x + 1][y + 1];
    tl = data[x - 1][y + 1];
    br = data[x + 1][y - 1];
    bl = data[x - 1][y - 1];

    // Right, left, top, and bottom
    dataCell r, b, t, l;
    r = data[x + 1][y];
    l = data[x - 1][y];
    t = data[x][y + 1];
    b = data[x][y - 1];

    if (x + 2 < sizeX && r.voronoi == occupied) {
      // Fill to the right
      if (tr.voronoi != occupied && br.voronoi != occupied && data[x + 2][y].voronoi != occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x + 1, y));
        // Add to the open priority queue
        data[x + 1][y] = r;
      }
    }
  
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { 
      // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { 
      // r==retry
      c.voronoi = voronoiRetry;
      // printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  // printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  // int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        // if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        // if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    // assert(voroCount<2);
    // if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}
