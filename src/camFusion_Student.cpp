
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

/* STUDENT INCLUDES START */
#include <opencv2/core.hpp>

/* STUDENT INCLUDES END   */

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    // Step 1: Calculate the mean euclidean distance from all current matched keypoints within the region of interest in the camera image
    float dist_mean_euclidean = 0.0;
    float size = 0.0;

    for (auto pair_match : kptMatches) {
        // Retrieve the previous and current keypoint from each match
        const auto& pt_prev = kptsPrev[pair_match.queryIdx].pt;
        const auto& pt_curr = kptsCurr[pair_match.trainIdx].pt;

        if (boundingBox.roi.contains(pt_curr)) {
            dist_mean_euclidean += cv::norm(pt_curr - pt_prev);
            size++;
        }
    }

    dist_mean_euclidean /= size;

    // Step 2: Add matched keypoints to the vector
    //         Based on the distance between previous and current matched keypoints, filter out those are too far away from the mean with threshold
    
    float factor_threshold = 1.2;

    for (auto pair_match : kptMatches) {
        // Retrieve the previous and current keypoint from each match
        const auto& pt_prev = kptsPrev[pair_match.queryIdx].pt;
        const auto& pt_curr = kptsCurr[pair_match.trainIdx].pt;

        if (boundingBox.roi.contains(pt_curr) && (cv::norm(pt_curr - pt_prev) < dist_mean_euclidean * factor_threshold)) {
            boundingBox.kptMatches.push_back(pair_match);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    // double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    // double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);

    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    // compute median dist. ratio to remove outlier influence
    double medDistRatio = distRatios.size() %2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

    double dT = 1 / frameRate;
    TTC = - dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    // auxiliary variables
    double dT = 1.0 / frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find all Lidar points within ego lane    
    vector<double> d_Prev, d_Curr;
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
      if (abs(it->y) <= laneWidth/2.0) {
        d_Prev.push_back(it->x);
      }
      
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
      if (abs(it->y) <= laneWidth/2.0) {
        d_Curr.push_back(it->x);
      }
    }

    // Take subset from previous Lidar points
    // 1/3 to 2/3 subset from the vector
    int idx_Prev_Start = d_Prev.size() / 3;
    int idx_Prev_End   = d_Prev.size() / 3 * 2;
    sort(d_Prev.begin(), d_Prev.end());

    // cout << "The size of d_Prev is " << d_Prev.size() << endl;
    // cout << "The start index of d_Prev is " << idx_Prev_Start << endl;
    // cout << "The end index of d_Prev is " << idx_Prev_End << endl;

    // Calculate the average distance
    double d_avg_Prev = accumulate(d_Prev.begin() + idx_Prev_Start, d_Prev.begin() + idx_Prev_End, 0.0) / (idx_Prev_End - idx_Prev_Start);

    // cout << "The average of d_Prev is " << d_avg_Prev << endl;

    // Take subset from current Lidar points
    // 1/3 to 2/3 subset from the vector
    int idx_Curr_Start = d_Curr.size() / 3;
    int idx_Curr_End   = d_Curr.size() / 3 * 2;
    sort(d_Curr.begin(), d_Curr.end());

    // Calculate the average distance
    double d_avg_Curr = accumulate(d_Curr.begin() + idx_Curr_Start, d_Curr.begin() + idx_Curr_End, 0.0) / (idx_Curr_End - idx_Curr_Start);

    // compute TTC from both measurements
    TTC = d_avg_Curr * dT / (d_avg_Prev - d_avg_Curr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    // cv::DMatch Class Reference
    // https://docs.opencv.org/master/d4/de0/classcv_1_1DMatch.html
    // DMatch (int _queryIdx, int _trainIdx, float _distance)
    // queryIdx: query descriptor index (Previous keypoint index)
    // trainIdx: train descriptor index (Current keypoint index)

    // Step 1: Create a 2D table/map with previouse and current bounding box numbers
    // Initialize counter table with zeros (row and col are from the previouse and current bounding box numbers
    cv::Mat tbl_counter = cv::Mat::zeros(prevFrame.boundingBoxes.size(), currFrame.boundingBoxes.size(), CV_32S);

    // Step 2: Loop through all the matched keypoints in previouse and current bounding box numbers (Nested for loop)
    for (auto pair_match : matches) {
        // Retrieve the previous and current keypoint from each match
        const auto& pt_prev = prevFrame.keypoints[pair_match.queryIdx].pt;
        const auto& pt_curr = currFrame.keypoints[pair_match.trainIdx].pt;

        // Step 3: Loop through all the previouse and current bounding box numbers (Nested for loop)
        for (size_t i = 0; i < prevFrame.boundingBoxes.size(); i++) {
            for (size_t j = 0; j < currFrame.boundingBoxes.size(); j++) {
                // Step 4: If the previous keypoint in the ROI of previous bounding box
                //         and the current keypoint in the ROI of current bounding box,
                //         Increase the counter in the 2D table/map
                if (prevFrame.boundingBoxes[i].roi.contains(pt_prev) && currFrame.boundingBoxes[j].roi.contains(pt_curr)) {
                    tbl_counter.at<int> (i, j)++;
                }
            }
        }
    }

    // Using minMaxLoc here
    // https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#minmaxloc
    // maxLoc – pointer to the returned maximum location (in 2D case);
    double min = 0, max = 0;
    cv::Point minLoc, maxLoc;

    // Step 5: Loop through all the counter number in the 2D table/map (for loop)
    for (size_t i = 0; i < tbl_counter.rows; i++) {
        // Step 6: Find the maximum value index in each row
        // maxLoc contains coordinate of maximum value
        cv::minMaxLoc(tbl_counter.row(i), &min, &max, &minLoc, &maxLoc);
        
        // Step 7: If the maximum value is non-zero, place it into the bbBestMatches
        //         The maxLoc.x is the column index of the maximum value in each row
        if (max != 0) {
            bbBestMatches.emplace(i, maxLoc.x);
        }
        else {
            continue;
        }
    }
}
