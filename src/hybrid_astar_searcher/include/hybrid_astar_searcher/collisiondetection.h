#pragma once

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include "constants.h"
#include "lookup.h"
// #include "node2d.h"
#include "node3d.h"
#include "hybrid_astar_searcher/type_defs.h" 


namespace planning {
   /*
      \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.
      It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
   */
   class CollisionDetection {
      public:
         CollisionDetection();

      /*!
         \brief evaluates whether the configuration is safe
         \return true if it is traversable, else false
      */
      bool isTraversable(Vec3d pose) {
         /* Depending on the used collision checking mechanism this needs to be adjusted
            standard: collision checking using the spatial occupancy enumeration
            other: collision checking using the 2d costmap and the navigation stack
         */
         float cost = configurationTest(pose(0), pose(1), pose(2)) ? 0 : 1;
         return cost <= 0;
      }

      bool isTraversable(int X, int Y, float Theta) {
         float cost = configurationTest_(X, Y, Theta) ? 0 : 1;
         return cost <= 0;
      }

      /*!
         \brief updates the grid with the world map
      */
      void updateGrid(const nav_msgs::OccupancyGridConstPtr& map) {grid = map;}


         private:
      /*!
         \brief Calculates the cost of the robot taking a specific configuration q int the World W
         \param x the x position
         \param y the y position
         \param t the theta angle
         \return the cost of the configuration q of W(q)
         \todo needs to be implemented correctly
      */
      //   float configurationCost(float x, float y, float t) {return 0;}

      /*!
         \brief Tests whether the configuration q of the robot is in C_free
         \param x the x position
         \param y the y position
         \param t the theta angle
         \return true if it is in C_free, else false
      */
      bool configurationTest(float x, float y, float t);
      bool configurationTest_(int X, int Y, float Theta);

      private:
      nav_msgs::OccupancyGridConstPtr grid;
      // The collision lookup table
      Constants::config collisionLookup[Constants::headings * Constants::positions];
   
   };

}
#endif // COLLISIONDETECTION_H
