/*
 * @Author: Antonio Chan
 * @Date: 2021-04-04 21:27:51
 * @LastEditTime: 2021-04-04 21:37:04
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/ORBVocabulary.h
 */

#ifndef INC_ORBVOCABULARY_H_
#define INC_ORBVOCABULARY_H_

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2 {

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;

}  // namespace ORB_SLAM2

#endif  // INC_ORBVOCABULARY_H_
