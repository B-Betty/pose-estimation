//
// Created by betty on 2017/5/16.
//
#include <jni.h>
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/video/video.hpp>
#include <iostream>
#include <android/log.h>
#include<iostream>
#include <cwchar>
#include "list"



using namespace cv;
using namespace std;

extern "C"
{

JNIEXPORT jdoubleArray JNICALL
Java_com_example_betty_pose_1estimation_MainActivity_floatMatrixFromJNI(JNIEnv *env,
                                                                        jobject instance,
                                                                        jlong nativeObjAddr,
                                                                        jlong nativeObjAddr1);

JNIEXPORT jdoubleArray JNICALL
Java_com_example_betty_pose_1estimation_MainActivity_floatMatrixFromJNI(JNIEnv *env,
                                                                        jobject instance,
                                                                        jlong nativeObjAddr,
                                                                        jlong nativeObjAddr1) {
/**
 * ORB算法求取相机的旋转矩阵
 */
//    // TODO
//    cv::Mat descriptors_1, descriptors_2;
//    vector<cv::KeyPoint> keypoints_1, keypoints_2;
//    vector<cv::Point2d> points_1;
//    vector<cv::Point2d> points_2;
////    cv::Ptr<cv::FeatureDetector> orb_detector = cv::ORB::create();//特征类检测对象
//    cv::Ptr<cv::FeatureDetector> orb_detector = cv::FastFeatureDetector::create();//特征类检测对象
//    cv::Ptr<cv::DescriptorExtractor> orb_extractor = cv::ORB::create();
//
//    cv::BFMatcher matcher;//暴力匹配器
//    vector<cv::DMatch> matches;
//    cv::Mat R, t;
//    double max_dist = 0;
//    double min_dist = 10000;
//    cv::Mat &current_frame = *(cv::Mat *) nativeObjAddr;
//    cv::Mat &first_frame = *(cv::Mat *) nativeObjAddr1;
//    Mat tmpImg,tmpImg_first;
//    Size sz;
////    LOGI("camera processing");
////    ofstream fout("camera value.txt");
//
//    //压缩图片
////    pyrDown(first_frame, tmpImg_first, sz, BORDER_DEFAULT);
////    pyrDown(tmpImg_first, tmpImg_first, sz, BORDER_DEFAULT);
//
////    pyrDown(current_frame, tmpImg, sz, BORDER_DEFAULT);
////    pyrDown(tmpImg, tmpImg, sz, BORDER_DEFAULT);
//
//    orb_detector->detect(first_frame, keypoints_1);
//    orb_detector->detect(current_frame, keypoints_2);
//    //计算描述子
//    orb_extractor->compute(first_frame, keypoints_1, descriptors_1);
//    orb_extractor->compute(current_frame, keypoints_2, descriptors_2);
//    matcher.match(descriptors_1, descriptors_2, matches);
//
//    for (int i = 0; i < descriptors_1.rows; i++) {
//        double dist = matches[i].distance;
//        if (dist < min_dist)
//            min_dist = dist;
//        if (dist > max_dist)
//            max_dist = dist;
//    }
//    vector<cv::DMatch> good_matches;
//    for (int i = 0; i < descriptors_1.rows; i++) {
//        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
//            good_matches.push_back(matches[i]);
//        }
//    }
//    double K[3][3] = {3133.61, 0, 2104, 2104, 0, 3102.27, 0, 0, 1};
//
//    //将匹配点转换为vector<Point2d>形式
//    for (int i = 0; i < good_matches.size(); i++) {
//        points_1.push_back(keypoints_1[good_matches[i].queryIdx].pt);
//        points_2.push_back(keypoints_2[good_matches[i].queryIdx].pt);
//        circle(current_frame, points_2.at(i), 1 , Scalar(10,10,255), 2, 8, 0);
//    }
//
//    //计算本质矩阵可以自行SVD加快处理速度
//    cv::Point2d principal_point(K[0][2], K[1][2]);//相机光心
//    double focal_length = 0.5 * (K[0][0] + K[1][1]);//相机焦距
//    cv::Mat essential_matrix;
//    essential_matrix = findEssentialMat(points_1, points_2, focal_length, principal_point);
//    //从本质矩阵恢复旋转和平移信息 t是默认归一化的矩阵
//    recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);

/**
 * 光流法获取相机的旋转矩阵
 */
    // TODO
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::Point2f> points_1;
    vector<cv::Point2f> points_2;
    cv::Mat R, t;
    cv::Mat &current_frame = *(cv::Mat *) nativeObjAddr;
    cv::Mat &first_frame = *(cv::Mat *) nativeObjAddr1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    unsigned int maxCout = 300;//定义最大个数
    double minDis = 10;//定义最小距离
    double qLevel = 0.01;//定义质量水平
    //压缩图片
    Mat tmpImg, tmpImg_first;
    Size sz;
    pyrDown(first_frame, tmpImg_first, sz, BORDER_DEFAULT);
    pyrDown(tmpImg_first, tmpImg_first, sz, BORDER_DEFAULT);
    pyrDown(current_frame, tmpImg, sz, BORDER_DEFAULT);
    pyrDown(tmpImg, tmpImg, sz, BORDER_DEFAULT);

    //Fast提取角点
//    FAST(tmpImg_first, keypoints_1, fast_threshold, nonmaxSuppression);
//    KeyPoint:: convert(keypoints_1, points_1, vector<int>());

    goodFeaturesToTrack(tmpImg_first, points_1, maxCout, qLevel, minDis);
    vector<uchar> status;
    vector<float> err;
    status.reserve(maxCout);
    err.reserve(maxCout);
    Size winSize = Size(13, 13);
    TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
//    Mat gray_first, gray_current;
//    first_frame.convertTo(gray_first, CV_8U);
//    current_frame.convertTo(gray_current, CV_8U);
// getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    if (points_1.empty()) {
        return 0;
    }
    calcOpticalFlowPyrLK(tmpImg_first, tmpImg, points_1, points_2, status, err, winSize, 3,
                         termcrit, 0, 0.001);
    unsigned int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++) {
        Point2f pt = points_2.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }
            points_1.erase(points_1.begin() + (i - indexCorrection));
            points_2.erase(points_2.begin() + (i - indexCorrection));
            indexCorrection++;
        }

        double K[3][3] = {412.1066, 0, 188.7078, 0, 413.9122, 259.5036, 0, 0, 1};
        for (int i = 0; i < points_2.size(); i++) {
            circle(tmpImg, points_2.at(i), 1, Scalar(10, 10, 255), 1, 8, 0);
        }
        //计算本质矩阵可以自行SVD加快处理速度
        cv::Point2d principal_point(K[0][2], K[1][2]);//相机光心
        double focal_length = 0.5 * (K[0][0] + K[1][1]);//相机焦距
        cv::Mat essential_matrix, mask;
        if (points_2.empty()) {
            return 0;
        }
        essential_matrix = findEssentialMat(points_1, points_2, focal_length, principal_point,
                                            RANSAC, 0.999, 1.0, mask);
        if (essential_matrix.cols != 3 || essential_matrix.rows != 3) {
            return 0;
        }
        //从本质矩阵恢复旋转和平移信息 t是默认归一化的矩阵
        recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point,
                    mask);
        if ((!R.data) && (!t.data)) {
            return NULL;
        }
        jdoubleArray resultarray = env->NewDoubleArray((R.rows + 1) * R.cols);
        jdouble *element;
        element = env->GetDoubleArrayElements(resultarray, JNI_FALSE);
        for (int i = 0; i < R.rows; i++) {
            for (int j = 0; j < R.cols; j++) {
                element[i * R.cols + j] = R.at<double>(i, j);
            }
        }
        for (int m = 0; m < t.rows; m++) {
            for (int n = 0; n < t.cols; n++) {
                element[9 + (m * t.cols + n)] = t.at<double>(m, n);
            }
        }
        pyrUp(tmpImg, tmpImg, sz, BORDER_DEFAULT);
        pyrUp(tmpImg, current_frame, sz, BORDER_DEFAULT);
        pyrUp(tmpImg_first, tmpImg_first, sz, BORDER_DEFAULT);
        pyrUp(tmpImg_first, first_frame, sz, BORDER_DEFAULT);
        env->ReleaseDoubleArrayElements(resultarray, element, 0);
        return resultarray;
    }
}
}