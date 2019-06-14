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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

//ExtractorNode 使得最后的提取出来的特征点分布均匀，所以在预计对所有 lNodes 中的 ExtractorNode 
//进行分裂能够得到足够多的特征点时（lNodes.size() + nToExpand * 3 > N），
//优先分裂点数多的 ExtractorNode。如果点数够了，就不分裂点数少的 ExtractorNode。   
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:
    //机选图像金字塔
    void ComputePyramid(cv::Mat image);
    //通过八叉树的方式计算关键点
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);  
    //通过八叉树的方式分配关键点
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    //用于存放训练的模板
    std::vector<cv::Point> pattern;
    
    //特征的数目
    int nfeatures;
    //尺度因子
    double scaleFactor;
    //金字塔的层数
    int nlevels;
    //初始的ｆａｓｔ角点阈值
    int iniThFAST;
    //最小的ｆａｓｔ角点阈值
    int minThFAST;
    //每一层金字塔的特征数目
    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;
    //每一层的尺度因子，相邻两层的比值为scaleFactor，０层的尺度因子为１；
    std::vector<float> mvScaleFactor;
    //每一层尺度因子的倒数
    std::vector<float> mvInvScaleFactor; 
    //每一层尺度因子的平方
    std::vector<float> mvLevelSigma2;
    //每一层尺度因子平方的倒数
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

