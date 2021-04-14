/*
 * @Author: Antonio Chan
 * @Date: 2021-04-06 16:56:50
 * @LastEditTime: 2021-04-14 14:48:33
 * @LastEditors: Antonio Chan
 * @Description: Modified from ORB-SLAM2
 * @FilePath: /Localization/inc/Map.h
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

// 公用库.
#include <mutex>
#include <set>

// ORB-SLAM2中的其他模块.
#include "KeyFrame.h"
#include "MapPoint.h"

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
class MapPoint;
class KeyFrame;

// 地图数据类型.
class Map {
 public:
  /**
   * @brief 构造函数
   * @note
   */
  Map();

  /**
   * @brief 向地图中添加关键帧
   * @note
   * @param pKF: 关键帧
   * @return None
   */
  void AddKeyFrame(KeyFrame *pKF);

  /**
   * @brief 向地图中添加地图点
   * @note
   * @param pMP: 地图点
   * @return None
   */
  void AddMapPoint(MapPoint *pMP);

  /**
   * @brief 从地图中擦除地图点
   * @note
   * @param pMP: 地图点
   * @return None
   */
  void EraseMapPoint(MapPoint *pMP);

  /**
   * @brief 从地图中删除关键帧
   * @note
   * 实际上这个函数中目前仅仅是删除了在std::set中保存的地图点的指针,并且删除后
   * 之前的地图点所占用的内存其实并没有得到释放
   * @param pKF: 关键帧
   * @return None
   */
  void EraseKeyFrame(KeyFrame *pKF);

  /**
   * @brief 设置参考地图点
   * @note
   * 一般是指,设置当前帧中的参考地图点;
   * 这些点将用于DrawMapPoints函数画图
   * @param vpMPs: 地图点们
   * @return None
   */
  void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

  /**
   * @brief 这个函数好像没有被用到过
   * @note REVIEW
   * @return None
   */
  void InformNewBigChange();

  /**
   * @brief 获取最大改变;但是这个函数最终好像并没有被使用到
   * @note
   * @return (int) #TODO
   */
  int GetLastBigChangeIdx();

  /**
   * @brief 获取地图中的所有关键帧
   * @note
   * @return (std::vector<KeyFrame*>) 获得的关键帧序列
   */
  std::vector<KeyFrame *> GetAllKeyFrames();

  /**
   * @brief 获取地图中的所有地图点
   * @note
   * @return (std::vector<MapPoint*>) 获得的地图点序列
   */
  std::vector<MapPoint *> GetAllMapPoints();

  /**
   * @brief 获取地图中的所有参考地图点
   * @note
   * @return (std::vector<MapPoint*>) 获得的参考地图点序列
   */
  std::vector<MapPoint *> GetReferenceMapPoints();

  /**
   * @brief 获得当前地图中的地图点个数
   * @note
   * @return (long unsigned int) 个数
   */
  long unsigned int MapPointsInMap();

  /**
   * @brief 获取当前地图中的关键帧个数
   * @note
   * @return (long unsigned) 关键帧个数
   */
  long unsigned KeyFramesInMap();

  /**
   * @brief 获取关键帧的最大id
   * @note
   * @return (long unsigned int) id
   */
  long unsigned int GetMaxKFid();

  /**
   * @brief 清空地图
   * @note
   * @return None
   */
  void clear();

 public:
  // (vector<KeyFrame *>) #TODO
  // 保存了最初始的关键帧
  vector<KeyFrame *> mvpKeyFrameOrigins;

  // (std::mutex) #TODO
  //当更新地图时的互斥量.回环检测中和局部BA后更新全局地图的时候会用到这个
  std::mutex mMutexMapUpdate;

  // (std::mutex) #TODO
  // This avoid that two points are created simultaneously in separate threads
  // (id conflict)
  //为了避免地图点id冲突设计的互斥量
  std::mutex mMutexPointCreation;

 protected:
  // (std::set<MapPoint *>) #TODO
  // 存储所有的地图点
  std::set<MapPoint *> mspMapPoints;

  // (std::set<KeyFrame *>) #TODO
  // 存储所有的关键帧
  std::set<KeyFrame *> mspKeyFrames;

  // (std::vector<MapPoint *>) #TODO
  //参考地图点
  std::vector<MapPoint *> mvpReferenceMapPoints;

  // (long unsigned int) #TODO
  //当前地图中具有最大ID的关键帧
  long unsigned int mnMaxKFid;

  // (int) #TODO
  // 貌似在程序中并没有被使用过
  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;

  // (std::mutex) #TODO
  //类的成员函数在对类成员变量进行操作的时候,防止冲突的互斥量
  std::mutex mMutexMap;
};

}  // namespace ORB_SLAM2

#endif  // INC_MAP_H_
