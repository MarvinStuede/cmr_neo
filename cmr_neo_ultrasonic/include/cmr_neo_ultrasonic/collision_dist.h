/* *****************************************************************
Copyright (c) 2019, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
 */
#ifndef COLLISION_DIST_H
#define COLLISION_DIST_H

/**
 * @brief Struct to describe multiplicative velocity limits
 * @details Struct to describe multiplicative velocity limits.
 * Factors for positive/negative x and angular direction. Factor = 1 --> no decrease, Factor = 0 --> full decrease
 * Factor between 0 and 1 --> multiply with velocity to decrease
 */
struct VelLimits{
  double posX;
  double negX;
  double posAngular;
  double negAngular;
};

/**
 * @brief Struct for multiplicative velocity decrease
 * @details Struct to describe multiplicators to decrease the velocity for different degrees of freedom.
 * Factors for positive/negative x and angular direction. Factor = 1 --> no decrease, Factor = 0 --> full decrease
 * Factor between 0 and 1 --> multiply with velocity to decrease
 */
struct MaxVelFactor {

  MaxVelFactor() { }

  MaxVelFactor(double negX, double posX, double negAngular, double posAngular){

    limits_.negX = negX;
    limits_.posX = posX;
    limits_.negAngular = negAngular;
    limits_.posAngular = posAngular;
    limitsBin_ = limits_;
  }
  void setLimits(VelLimits newLimits){
    limits_ = newLimits;
  }

  VelLimits getLimits(){return limits_;}

  VelLimits limits_;
  VelLimits limitsBin_;
  /**
 * @brief Merge two to one MaxVelFactor object
 * @param  first  First Object
 * @param  second Second Object
 * @return        Merged object
 */
  static MaxVelFactor mergeObjects(const MaxVelFactor &first, const MaxVelFactor &second) {

    MaxVelFactor result(1,1,1,1);

    //To merge the to objects, the minimum value of each object is used for the resulting object
    result.limits_.negX = std::min(first.limits_.negX, second.limits_.negX);
    result.limits_.posX = std::min(first.limits_.posX, second.limits_.posX);
    result.limits_.negAngular = std::min(first.limits_.negAngular, second.limits_.negAngular);
    result.limits_.posAngular = std::min(first.limits_.posAngular, second.limits_.posAngular);

    return result;
  }
  /**
 * @brief Merge a vector of MaxVelFactor objects to one
 * @param  maxVelocieties Vector of MaxVelFactor objects
 * @return                Merged object
 */
  static MaxVelFactor mergeObjects(const std::vector<MaxVelFactor> &maxVelocieties) {

    MaxVelFactor mergedVelocities(1,1,1,1);

    for(const auto &maxVel : maxVelocieties)
      mergedVelocities = MaxVelFactor::mergeObjects(mergedVelocities, maxVel);

    return mergedVelocities;
  }
};
#endif // COLLISION_DIST_H
