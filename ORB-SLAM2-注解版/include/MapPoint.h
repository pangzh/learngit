/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

/**
 * @brief MapPoint是一个地图点
 */
class MapPoint
{
public:
  
    ///构造函数，两种，一种是根据关键帧来构造构造ｍａｐpoint,另一种是根据普通帧来构造。  
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    ///下面两个函数分别是设定地图点的世界坐标和读取它的世界坐标
    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();
    
    ///获取点的法向量和参考关键帧
    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();
    
    ///第一个函数是获取能观测到该点的所有关键帧，函数返回一个容器，容器里的元素是关键帧指针和该地图点在该关键帧中对应的索引号。
    ///第二个函数应该是返回能看到该地图点的帧的数量
    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();
    
    ///第一个函数是为该地图点添加一个观测者，即将关键帧的指针和索引号添加到某个ｍａｐ容器中。
    ///为该地图点提出一个观测者，这种情况可能发生在关键帧被剔除了，或者由于优化后地图点和关键帧位姿发生了变化，导致地图点的观测性发生了变化
    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);
    
    ///第一个函数应该是获取该地图点在关键帧中的索引
    ///第二个函数判断在地图点是否在指定的关键帧中
    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);
    
    //第一个函数是用来删除与被剔除的地图点有关的量的，具体可以看代码；它由变量bBad（该变量默认是ｆａｌｓｅ,但是当该地图点能被少于３个关键帧看到时，变成ｔｒｕｅ）
    void SetBadFlag();
    bool isBad();
    
    ///第一个函数是用一个新的地图点来代替该地图点，即所谓的地图点的融合
    void Replace(MapPoint* pMP);
    MapPoint* GetReplaced();
    
    ///第一个函数是增加该地图点的观测次数，表示理论应该被观测到的帧的个数
    ///第二个函数是增加该地图点的发现次数，表示实际上被观测到的帧的个数
    ///第三个函数返回该地图点实际被观测到的帧数占理论的帧数的比例，小于２５％的地图点会被剔除
    ///第四个内联函数返回实际观测到该点的帧的个数，小于三个的地图点会被剔除
    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }
    
    ///计算地图点的代表描述子
    void ComputeDistinctiveDescriptors();
    ///该函数返回地图点的描述子
    cv::Mat GetDescriptor();
    
    ///更新地图点的法向量和深度
    void UpdateNormalAndDepth();
    
    ///第一个函数返回该地图点的最小不变性距离ｄmin
    ///第二个函数返回地图点的最大不变性距离ｄmax
    ///第三四个函数是根据地图点的当前距离来计算尺度，目前还不明白这样做的目的和具体做法
    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    long unsigned int mnId; ///< Global ID for MapPoint
    static long unsigned int nNextId;	///静态值，所有MapPoint类实例共享一直值，它的值加１就是当前创建的地图点的全局ｉｄ.即ｍｎId.
    const long int mnFirstKFid; ///< 创建该MapPoint的关键帧ID
    const long int mnFirstFrame; ///< 创建该MapPoint的帧ID（即每一关键帧有一个帧ID）
    
    //能观测到该地图点的相机个数，小于３事，地图点被剔除
    int nObs;
    
    // Variables used by the tracking
    //下面的变量用于局部地图的跟踪，在 Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)里设定。每一次跟踪局部地图，对应地图点的这些变量被更新
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    // TrackLocalMap - SearchByProjection中决定是否对该点进行投影的变量
    // mbTrackInView==false的点有几种：
    // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
    // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
    // c 不在当前相机视野中的点（即未通过isInFrustum判断）
    bool mbTrackInView;//决定该地图点是否在局部地图中被跟踪，即是否投影到当前帧.在Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)函数里对这个值赋值，表明它能否被跟踪
    // TrackLocalMap - UpdateLocalPoints中防止将MapPoints重复添加至mvpLocalMapPoints的标记
    long unsigned int mnTrackReferenceForFrame;
    // TrackLocalMap - SearchLocalPoints中决定是否进行isInFrustum判断的变量
    // mnLastFrameSeen==mCurrentFrame.mnId的点有几种：
    // a 已经和当前帧经过匹配（TrackReferenceKeyFrame，TrackWithMotionModel）但在优化过程中认为是外点
    // b 已经和当前帧经过匹配且为内点，这类点也不需要再进行投影
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;//闭环时传播位姿的关键帧（闭环的始端关键帧）
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:

    // Position in absolute coordinates
    cv::Mat mWorldPos; ///< MapPoint在世界坐标系下的坐标

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations; ///< 观测到该MapPoint的KF和该MapPoint在KF中的索引,即关键中的特征跟这个地图点匹配

    // Mean viewing direction
    // 该MapPoint平均观测方向
    cv::Mat mNormalVector;
    // Best descriptor to fast matching
    // 每个3D点也有一个descriptor
    // 如果MapPoint与很多帧图像特征点对应（由keyframe来构造时），那么距离其它描述子的平均距离最小的描述子是最佳描述子
    // MapPoint只与一帧的图像特征点对应（由frame来构造时），那么这个特征点的描述子就是该3D点的描述子
    cv::Mat mDescriptor; ///< 通过 ComputeDistinctiveDescriptors() 得到的最优描述子

    // Reference KeyFrame
    KeyFrame* mpRefKF;

    // Tracking counters
    ///理论应该被观测到的帧数
    int mnVisible;
    ///实际被观测到的帧数
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;	///当能观测到该点的关键帧个数少于３时，该变量变ｔｒｕｅ
    MapPoint* mpReplaced;

    // Scale invariance distances
    ///分别是该地图点的最小尺度不变距离和最大尺度不变距离
    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;
    ///两个互斥量，分别用来防止多线程同时修改地图点位置和地图点特征
    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
