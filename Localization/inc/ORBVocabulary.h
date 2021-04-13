/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-13 09:54:25
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/ORBVocabulary.h
 */

#ifndef INC_ORBVOCABULARY_H_
#define INC_ORBVOCABULARY_H_

// 公用库.
#include "third_party/DBoW2/DBoW2/FORB.h"
#include "third_party/DBoW2/DBoW2/TemplatedVocabulary.h"

/**==============================================
 * *                   变量命名规则
 *   类的公有成员变量(public)的名称带有前缀m.
 *   类的私有成员变量(private)的名称前缀为空.
 *   枚举变量(enum)的名称带有前缀e.
 *   指针变量(pointer)的名称带有前缀p.
 *   线程变量(thread)的名称带有前缀t.
 *
 *   前缀先后顺序: m>p>t
 *
 *=============================================**/

namespace ORB_SLAM2 {
// 定义变量类型别名 DBoW2TD,
// TODO
typedef DBoW2::FORB::TDescriptor DBoW2TD;

// 定义变量类型别名 ORBVocabulary,
// 模板类 DBoW2::TemplatedVocabulary 中:
// 第一个实例化参数是 #TODO,
// 第二个实例化参数是 #TODO.
typedef DBoW2::TemplatedVocabulary<DBoW2TD, DBoW2::FORB> ORBVocabulary;

}  // namespace ORB_SLAM2

#endif  // INC_ORBVOCABULARY_H_
