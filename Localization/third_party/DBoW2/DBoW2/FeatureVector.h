/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 10:09:28
 * @LastEditTime: 2021-04-04 21:35:52
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/third_party/DBoW2/DBoW2/FeatureVector.h
 */
/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef THIRD_PARTY_DBOW2_DBOW2_FEATUREVECTOR_H_
#define THIRD_PARTY_DBOW2_DBOW2_FEATUREVECTOR_H_

#include <iostream>
#include <map>
#include <vector>

#include "BowVector.h"

namespace DBoW2 {

/// Vector of nodes with indexes of local features
class FeatureVector : public std::map<NodeId, std::vector<unsigned int> > {
 public:
  /**
   * Constructor
   */
  FeatureVector(void);

  /**
   * Destructor
   */
  ~FeatureVector(void);

  /**
   * Adds a feature to an existing node, or adds a new node with an initial
   * feature
   * @param id node id to add or to modify
   * @param i_feature index of feature to add to the given node
   */
  void addFeature(NodeId id, unsigned int i_feature);

  /**
   * Sends a string versions of the feature vector through the stream
   * @param out stream
   * @param v feature vector
   */
  friend std::ostream &operator<<(std::ostream &out, const FeatureVector &v);
};

}  // namespace DBoW2

#endif  // THIRD_PARTY_DBOW2_DBOW2_FEATUREVECTOR_H_
