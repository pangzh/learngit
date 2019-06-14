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
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();//请求线程停止
    void RequestReset();
    bool Stop();//停止线程
    void Release();
    bool isStopped();//判断是否已经停止
    bool stopRequested();//判断是否被要求停止
    bool AcceptKeyFrames();//判断是否接受关键帧
    void SetAcceptKeyFrames(bool flag);//设置是否接受关键帧
    bool SetNotStop(bool flag);//设置线程能否被停止，返回ｂｏｏｌ显示设置是否成功

    void InterruptBA();//打断局部建图线程的ＢＡ优化

    void RequestFinish();
    bool isFinished();
    //查询未处理的关键帧队列的长度
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();//查看列表中是否有等待被插入的关键帧
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();
    //剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入的质量不好的MapPoints
    void MapPointCulling();
    //检查并融合当前关键帧与相邻关键帧（两级相邻）重复的MapPoints
    void SearchInNeighbors();
    //剔除关键帧
    void KeyFrameCulling();
    //根据两关键帧的姿态计算两个关键帧之间的基本矩阵
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;
    //是否请求了重置
    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    // Tracking线程向LocalMapping中插入关键帧是先插入到该队列中
    std::list<KeyFrame*> mlNewKeyFrames; ///< 等待处理的关键帧列表

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
