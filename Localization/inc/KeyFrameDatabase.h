/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-12 22:20:48
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/KeyFrameDatabase.h
 */

#ifndef INC_KEYFRAMEDATABASE_H_
#define INC_KEYFRAMEDATABASE_H_

// 公用库.
#include <list>
#include <mutex>
#include <set>
#include <vector>

// ORB-SLAM2中的其他模块.
#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"

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
// 需要使用到的其他模块的前置声明.
class KeyFrame;
class Frame;

// 关键帧数据库数据类型.
class KeyFrameDatabase {
 public:
  /**
   * @brief 构造函数
   * @note
   * @param voc: 词袋模型的字典
   */
  KeyFrameDatabase(const ORBVocabulary &voc);

  /**
   * @brief 根据关键帧的词包, 更新数据库的倒排索引
   * @note
   * @param pKF: 关键帧
   * @return None
   */
  void add(KeyFrame *pKF);

  /**
   * @brief 删除关键帧
   * @note 关键帧被删除后, 更新数据库的倒排索引
   * @param pKF: 关键帧
   * @return None
   */
  void erase(KeyFrame *pKF);

  /**
   * @brief 清空关键帧数据库
   * @note
   * @return None
   */
  void clear();

  /**
   * @brief 在闭环检测中找到与该关键帧可能闭环的关键帧
   * @note
   * @param pKF: 需要闭环的关键帧
   * @param minScore: 相似性分数最低要求
   * @return (std::vector<KeyFrame *>) 可能闭环的关键帧
   */
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

  /**
   * @brief Relocalization
   * @note 在重定位中找到与该帧相似的关键帧
   * 1. 找出和当前帧具有公共单词的所有关键帧
   * 2. 只和具有共同单词较多的关键帧进行相似度计算
   * 3. 将与关键帧相连(权值最高)的前十个关键帧归为一组, 计算累计得分
   * 4. 只返回累计得分较高的组中分数最高的关键帧
   * @param F: 需要重定位的帧
   * @return 相似的关键帧
   */
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

 protected:
  // (const ORBVocabulary *) #TODO
  // Associated vocabulary
  // 预先训练好的词典
  const ORBVocabulary *mpVoc;

  // (std::vector<list<KeyFrame *>>) #TODO
  // Inverted file
  // 倒排索引, mvInvertedFile[i]表示包含了第i个word id的所有关键帧
  std::vector<list<KeyFrame *>> mvInvertedFile;

  // (std::mutex) #TODO
  // Mutex, 多用途的
  std::mutex mMutex;
};

}  // namespace ORB_SLAM2

#endif  // INC_KEYFRAMEDATABASE_H_
