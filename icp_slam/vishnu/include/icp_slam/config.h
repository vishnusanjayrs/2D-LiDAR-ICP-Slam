/** @file config.h */
//
// Created by rakesh on 27/08/18.
//

#ifndef ICP_SLAM_CONFIG_H
#define ICP_SLAM_CONFIG_H

// ICP parameters
#define MIN_ICP_CORRESPONDENCES 5

// mapping parameters
#define NUM_MAPPING_THREADS 4
#define NUM_FREE_SPACE_CELLS_SKIPPED 4
// how big is the obstacle given by each laser beam (in pixel)
#define LASER_BEAM_WIDTH 2

#endif //ICP_SLAM_CONFIG_H
