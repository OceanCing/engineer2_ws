//
// Created by koonyu on 23-12-3.
//

#include "../include/deep_exchanger/deep_exchanger.h"

namespace deep_exchanger
{
    void DeepExchanger::threading()
    {
        box_result_vec_.clear();
        side_triangle_vec_.clear();

        std::thread thread_1(std::bind(&DeepExchanger::modelProcess, this, std::ref(cv_image_->image)));
        std::thread thread_2(std::bind(&DeepExchanger::cvProcess, this, std::ref(cv_image_->image)));
        thread_1.join();
        if (thread_2.joinable())
            thread_2.join();

        if (box_result_vec_.empty() && side_triangle_vec_.empty())
        {
            poseNonSensePnP();
            return;
        }

        if (!(side_triangle_vec_.empty()))
        {
            std::array<std::pair<cv::Point, int>, 8> sorted_triangle_vec;
            // The order of points is the right-hand spiral(anti-clockwise).
            if ((side_triangle_vec_[0][0] - side_triangle_vec_[0].back()).cross(side_triangle_vec_[0][1] - side_triangle_vec_[0][0]) > 0)
            {
                sorted_triangle_vec[0].second = 1;
                sorted_triangle_vec[4].second = 1;
            }
            else
            {
                sorted_triangle_vec[0].second = 0;
                sorted_triangle_vec[4].second = 0;
            }
            sorted_triangle_vec[0].first = side_triangle_vec_[0][0];
            sorted_triangle_vec[4].first = side_triangle_vec_[0][0];
            for (int i = 0; i < side_triangle_vec_[0].size() - 2; i++)
            {
                if ((side_triangle_vec_[0][i + 1] - side_triangle_vec_[0][i]).cross(side_triangle_vec_[0][i + 2] - side_triangle_vec_[0][i + 1]) > 0)
                {
                    sorted_triangle_vec[i + 1].second = 1;
                    sorted_triangle_vec[i + 5].second = 1;
                }
                else
                {
                    sorted_triangle_vec[i + 1].second = 0;
                    sorted_triangle_vec[i + 5].second = 0;
                }
                sorted_triangle_vec[i + 1].first = side_triangle_vec_[0][i + 1];
                sorted_triangle_vec[i + 5].first = side_triangle_vec_[0][i + 1];
            }
            if ((side_triangle_vec_[0][3] - side_triangle_vec_[0][side_triangle_vec_[0].size() - 2]).cross(side_triangle_vec_[0][0] - side_triangle_vec_[0].back()) > 0)
            {
                sorted_triangle_vec[3].second = 1;
                sorted_triangle_vec[7].second = 1;
            }
            else
            {
                sorted_triangle_vec[3].second = 0;
                sorted_triangle_vec[7].second = 0;
            }
            sorted_triangle_vec[3].first = side_triangle_vec_[0].back();
            sorted_triangle_vec[7].first = side_triangle_vec_[0].back();

            for(int i = 0; i < side_triangle_vec_[0].size(); i++)
            {
                if(sorted_triangle_vec[i].second==1)
                {
                    side_triangle_vec_.clear();
                    side_triangle_vec_.reserve(1);
                    std::vector<cv::Point> side_triangle_vec;
                    side_triangle_vec.emplace_back(sorted_triangle_vec[i].first);
                    side_triangle_vec.emplace_back(sorted_triangle_vec[i + 1].first);
                    side_triangle_vec.emplace_back(sorted_triangle_vec[i + 2].first);
                    side_triangle_vec.emplace_back(sorted_triangle_vec[i + 3].first);
                    side_triangle_vec_.emplace_back(side_triangle_vec);
                    break;
                }
            }

            for (int i = 0; i < side_triangle_vec_[0].size() ; i++)
                cv::putText(cv_image_->image,std::to_string(i + 1), side_triangle_vec_[0][i], 1, 3,cv::Scalar(0,255,0),1);
        }

        if (box_result_vec_.empty()) pose_signal_ = false;
        else pose_signal_ = true;

        tf::Transform real_transform;
        real_transform.setOrigin(tf::Vector3(x_offset_, y_offset_, z_offset_));
        tf2::Quaternion real_quaternion;
        real_quaternion.setRPY(roll_offset_,pitch_offset_,yaw_offset_);
        real_transform.setRotation(tf::Quaternion(real_quaternion.x(), real_quaternion.y(), real_quaternion.z(), real_quaternion.w()));

        tf_broadcaster_.sendTransform(tf::StampedTransform(real_transform, ros::Time::now(), "base_link", "real_world"));

        if (tf_update_)
        {
            if (pose_signal_)
            {
                std::vector<cv::Point2f> pixel_points;
                pixel_points.emplace_back(static_cast<int32_t>(box_result_vec_[0].x1 * 1440 / image_size_),
                                          static_cast<int32_t>(box_result_vec_[0].y1 * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(box_result_vec_[0].x2 * 1440 / image_size_),
                                          static_cast<int32_t>(box_result_vec_[0].y2 * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(box_result_vec_[0].x3 * 1440 / image_size_),
                                          static_cast<int32_t>(box_result_vec_[0].y3 * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(box_result_vec_[0].x4 * 1440 / image_size_),
                                          static_cast<int32_t>(box_result_vec_[0].y4 * 1080 / image_size_));
                cv::solvePnP(world_points_,pixel_points,camera_matrix_,distortion_coefficients_,exchanger_rvec_,exchanger_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
                shape_signal_ = true;

//                cv::projectPoints(tf_trans_points,exchanger_rvec_,exchanger_tvec_,camera_matrix_,distortion_coefficients_,imagePoints_);

                getPnP(exchanger_rvec_, exchanger_tvec_);
            }
            else
            {
                if (side_triangle_vec_[0][0].x < (side_triangle_vec_[0][1].x + side_triangle_vec_[0][3].x) / 2) direction_signal_ = true;
                else direction_signal_ = false;

                std::vector<cv::Point2f> pixel_points;
                pixel_points.emplace_back(static_cast<int32_t>(side_triangle_vec_[0][0].x * 1440 / image_size_),
                                          static_cast<int32_t>(side_triangle_vec_[0][0].y * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(side_triangle_vec_[0][1].x * 1440 / image_size_),
                                          static_cast<int32_t>(side_triangle_vec_[0][1].y * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(side_triangle_vec_[0][2].x * 1440 / image_size_),
                                          static_cast<int32_t>(side_triangle_vec_[0][2].y * 1080 / image_size_));
                pixel_points.emplace_back(static_cast<int32_t>(side_triangle_vec_[0][3].x * 1440 / image_size_),
                                          static_cast<int32_t>(side_triangle_vec_[0][3].y * 1080 / image_size_));
                if (direction_signal_)
                {
                    cv::solvePnP(arrow_right_points,pixel_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
                    shape_signal_ = true;
                    getPnP(arrow_rvec_, arrow_tvec_);
                }
                else
                {
                    cv::solvePnP(arrow_left_points,pixel_points,camera_matrix_,distortion_coefficients_,arrow_rvec_,arrow_tvec_,bool(),cv::SOLVEPNP_ITERATIVE);
                    shape_signal_ = true;
                    getPnP(arrow_rvec_, arrow_tvec_);
                }
            }
        }
        else
            poseNonSensePnP();
    }

    inline void DeepExchanger::modelProcess(const cv::Mat &image)
    {
        cv::Mat tmp_image = image.clone();
        detect(tmp_image, score_thresh_);
        selectboxes(image,box_result_vec_);
        drawBboxes(image, box_result_vec_);
    }

    double pointDist(cv::Point a,cv::Point b)
    {
      double res= sqrt(pow(a.x-b.x,2)+ pow(a.y-b.y,2));
      return res;
    }


    void DeepExchanger::cvProcess(const cv::Mat &image)
    {
        auto input_img = image;
        cv::Mat1b bin_img(input_img.size());

        (*color_enhance_)(input_img);
        (*convert_color_)(input_img, bin_img);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_size_, morph_size_),
                                               cv::Point(-1, -1));
        cv::morphologyEx(bin_img, bin_img, morph_type_, kernel, cv::Point(-1, -1), morph_iterations_);
        binary_publisher_.publish(
                cv_bridge::CvImage(std_msgs::Header(), "mono8", bin_img).toImageMsg());

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin_img, contours, cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

        for (auto &contour: contours)
        {
            std::vector<cv::Point2i> hull;
            cv::convexHull(contour, hull, true);
//            ROS_INFO("contourArea=%0.2f",cv::contourArea(hull));
            if (cv::matchShapes(hull, temp_triangle_hull_, cv::CONTOURS_MATCH_I2, 0) <= triangle_moment_bias_
            && cv::contourArea(hull) >= min_triangle_threshold_)
            {
              cv::RotatedRect rect;
              cv::Point2f box[4],output[4];
              cv::Mat dst=image.clone();
              cv::Mat iread=cv::imread(ros::package::getPath("deep_exchanger")+"/red_triangle.png");
              cv::Mat output_dst(iread.cols,iread.rows,CV_8UC3);
              output[0]=cv::Point2f (output_dst.rows,output_dst.cols);
              output[1]=cv::Point2f (0,output_dst.cols);
              output[2]=cv::Point2f (output_dst.rows,0);
              output[3]=cv::Point2f (0,0);
              for (int i=0;i<hull.size();i++)
              {
                rect=cv::minAreaRect(hull);
                rect.points(box);
                std::sort(box,box+4,
                          [](cv::Point2f a,cv::Point2f b)
                          {return a.y<b.y;});

                std::sort(box,box+2,
                          [](cv::Point2f a,cv::Point2f b)
                          {return a.x<b.x;});

                std::sort(box+2,box+4,
                          [](cv::Point2f a,cv::Point2f b)
                          {return a.x<b.x;});

                //对矩形顶点顺时针排序
                if (box[1].x-box[0].x>box[2].y-box[0].y)
                {
                  std::swap(box[3],box[1]);
                  std::swap(box[1],box[0]);
                  std::swap(box[0],box[2]);
                }

//                for(int j=0; j<4; j++)
//                {
//                  cv::line(dst, box[j], box[(j+1)%4], Scalar(255, 0, 0), 1, 8);  //绘制最小外接矩形每条边
//                  cv::putText(dst,std::to_string(j+1),box[j],1,3,Scalar(0,0,255),1);
//                }

              }

              double line1= pointDist(box[0],box[2]);
              double line2= pointDist(box[1],box[3]);
              double area=line1*line2/2;
//              ROS_INFO("contourArea=%0.2f",area);
              if (area<side_triangle_select_area_)
              {
                continue ;
              }

              cv::Mat m=cv::getAffineTransform(box,output);
              cv::warpAffine(dst,output_dst,m,cv::Size(output_dst.rows,output_dst.cols));

              cv::Mat gray;
              cv::cvtColor(output_dst,gray,CV_RGB2GRAY);
              cv::GaussianBlur(gray,gray,cv::Size(5,5),0,0);

              cv::Mat bind;
              cv::threshold(gray,bind,75,200,CV_THRESH_BINARY);
              cv::morphologyEx(bind,bind,CV_MOP_OPEN,kernel);

              cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)); // 十字核
              cv::dilate(bind,bind,kernel_dilate);

              cv::Rect select=cv::Rect(0,output_dst.rows/3,output_dst.cols/2,output_dst.rows/3);
              cv::Mat roi=bind(select);
              cv::Scalar mean=cv::mean(roi);
//              std::cout<<mean[0]<<std::endl;
              if(mean[0]<10)
              {
                cv::flip(output_dst,output_dst,1);
                cv::flip(bind,bind,1);
//                ROS_INFO("OK");
              }

//              cv::Mat result(output_dst.rows - iread.rows + 1, output_dst.cols - iread.cols + 1, CV_32FC1);
//              cv::matchTemplate(output_dst,iread,result,CV_TM_CCOEFF_NORMED);
//              double dMaxVal;
//              minMaxLoc(result, 0, &dMaxVal, 0, 0);
//              ROS_INFO("match=%0.2f",dMaxVal);
//              if(dMaxVal<0.25)
//              {
//                continue ;
//              }


//              cv::Mat canny;
//              cv::Canny(gray,canny,20,200,3);
//              cv::dilate(canny,canny,kernel);

              std::vector<cv::Vec4f> lines;
              std::vector<cv::Point> pointtotrans;
//              cv::HoughLinesP(canny,lines,1,CV_PI/180,50,75,20);

              Lsd_LineDetect(bind,lines);

              for (const cv::Vec4f& line : lines) {
                cv::Point2f pt1(line[0], line[1]); // 起点 (x1, y1)
                cv::Point2f pt2(line[2], line[3]); // 终点 (x2, y2)
                cv::circle(output_dst,pt1,2,Scalar(0,255,0),3);
                cv::circle(output_dst,pt2,2,Scalar(0,255,0),3);
                cv::line(output_dst, pt1, pt2, cv::Scalar(0, 0, 255), 2); // 用红色绘制直线段
                pointtotrans.push_back(pt1);
                pointtotrans.push_back(pt2);
              }

//              for (size_t i = 0; i < lines.size(); i++)
//              {
//                cv::Vec4i l= lines[i];
//
//                cv::Point pointToTransform(l[0], l[1]);
//                pointtotrans.push_back(pointToTransform);
//
//                cv::Point pointToTransform2(l[2], l[3]);
//                pointtotrans.push_back(pointToTransform2);
//              }

              cv::Mat im;
              cv::invertAffineTransform(m,im);

              cv::Point midpoint3;
              cv::Point transformp3;
//              cv::Point midpoint2,midpoint4;
//              cv::Point transformp2,transformp4;

              double num3x,num3y;
              int num3=0;
//              double num2x,num2y;
//              int num2=0;
//              double num4x,num4y;
//              int num4=0;
              std::vector<cv::Point> output_point(4);
              for(std::vector<cv::Point>::iterator it=pointtotrans.begin();it!=pointtotrans.end();it++)
              {
//                cv::circle(output_dst,cv::Point (it->x,it->y),2,Scalar(0,255,0),3);
                if(it->x < output_dst.cols/4)
                {
                  num3x=num3x+it->x;
                  num3y=num3y+it->y;
                  num3=num3+1;
//                  ROS_INFO("num3x=%0.2f,num3y=%0.2f,num=%d",num3x,num3y,num3);
                }
//                if(it->x > output_dst.cols*3/4 && it->y > output_dst.rows*3/4)
//                {
//                  num2x=num2x+it->x;
//                  num2y=num2y+it->y;
//                  num2=num2+1;
//                }
//                if(it->x > output_dst.cols*3/4 && it->y < output_dst.rows/4)
//                {
//                  num4x=num4x+it->x;
//                  num4y=num4y+it->y;
//                  num4=num4+1;
//                }
              }

//              midpoint2.x=num2x/num2;
//              midpoint2.y=num2y/num2;
              midpoint3.x=num3x/num3;
              midpoint3.y=num3y/num3;
//              midpoint4.x=num4x/num4;
//              midpoint4.y=num4y/num4;

              if(mean[0]<10)
              {
//                midpoint2.x=output_dst.cols-midpoint2.x;
                midpoint3.x=output_dst.cols-midpoint3.x;
//                midpoint4.x=output_dst.cols-midpoint4.x;
              }

//              transformp2.x = im.at<double>(0, 0) * midpoint2.x + im.at<double>(0, 1) * midpoint2.y + im.at<double>(0,2);
//              transformp2.y = im.at<double>(1, 0) * midpoint2.x + im.at<double>(1, 1) * midpoint2.y + im.at<double>(1,2);
              transformp3.x = im.at<double>(0, 0) * midpoint3.x + im.at<double>(0, 1) * midpoint3.y + im.at<double>(0,2);
              transformp3.y = im.at<double>(1, 0) * midpoint3.x + im.at<double>(1, 1) * midpoint3.y + im.at<double>(1,2);
//              transformp4.x = im.at<double>(0, 0) * midpoint4.x + im.at<double>(0, 1) * midpoint4.y + im.at<double>(0,2);
//              transformp4.y = im.at<double>(1, 0) * midpoint4.x + im.at<double>(1, 1) * midpoint4.y + im.at<double>(1,2);


//              output_point[1]=transformp2;
              output_point[2]=transformp3;
//              output_point[3]=transformp4;


              cv::approxPolyDP(contour, contour, triangle_approx_epsilon_, true);
//              cv::polylines(cv_image_->image, contour, true, cv::Scalar(255, 0, 255), 1);

              if (contour.size() < 4)
                continue;
              std::vector<cv::Point> out_approx(4);

              rm_cgal_tools_polygon_simplification::approxPolyCGAL(contour, out_approx, 4);

              int concave_amount = 0;
              ((out_approx[0] - out_approx.back()).cross(out_approx[1] - out_approx[0]) > 0) && (++concave_amount);
              for (int i = 0; i < out_approx.size() - 2; i++)
                    // The order of points is the right-hand spiral(anti-clockwise).
                    ((out_approx[i + 1] - out_approx[i]).cross(out_approx[i + 2] - out_approx[i + 1]) > 0) && (++concave_amount);
              ((out_approx.back() - out_approx[out_approx.size() - 2]).cross(out_approx[0] - out_approx.back()) > 0) && (++concave_amount);

              output_point[1]=out_approx[2];
              output_point[3]=out_approx[0];

//              ROS_INFO("contourArea=%0.2f",area);
              if(pointDist(output_point[2],output_point[1])<20 || pointDist(output_point[2],output_point[1])>600
                  || pointDist(output_point[2],output_point[3])<20 || pointDist(output_point[2],output_point[3])>600)
              {
                continue ;
              }

//              if (concave_amount==1)
//              {
//                for (int i = 0; i < out_approx.size()/2; i++)
//                {
//                  cv::circle(cv_image_->image, out_approx[2*i], 2, cv::Scalar(0, 255, 0), 2);
//                }
//                side_triangle_vec_.emplace_back(out_approx);
//              }

              cv::Point2f transformp1;
              float x2=output_point[1].x,y2=output_point[1].y,
                    x3=output_point[2].x,y3=output_point[2].y,
                    x4=output_point[3].x,y4=output_point[3].y;
              transformp1.x=x2+x4-x3;
              transformp1.y=y2+y4-y3;

              output_point[0]=transformp1;
              for (int t=0;t<4;t++)
              {
                cv::line(cv_image_->image,output_point[t],output_point[(t+1)%4],Scalar(0,255,0),1,8);
                cv::circle(cv_image_->image, output_point[t], 2, cv::Scalar(0, 255, 0), 2);
//                cv::putText(cv_image_->image,std::to_string(t+1),output_point[t],1,3,Scalar(0,0,255),3);
              }

              side_triangle_vec_.emplace_back(output_point);

              sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_dst).toImageMsg();
              trans_publisher_.publish(msg1);

              sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", bind).toImageMsg();
              hough_publisher_.publish(msg2);

            }
        }
    }

    void DeepExchanger::detect(cv::Mat image, double score_threshold)
    {
        InferenceEngine::Blob::Ptr input_blob = infer_request_.GetBlob(input_name_);

        preProcess(image, input_blob);

        // do inference
        infer_request_.StartAsync();
        infer_request_.Wait(InferenceEngine::IInferRequest::WaitMode::RESULT_READY);

        // get output
        std::vector<std::vector<BoxInfo>> results;
        results.resize(this->num_class_);

        for (const auto &head_info : this->heads_info_)
        {
            const InferenceEngine::Blob::Ptr dis_pred_blob = infer_request_.GetBlob(head_info.dis_layer);
            const InferenceEngine::Blob::Ptr cls_pred_blob = infer_request_.GetBlob(head_info.cls_layer);

            auto mdis_pred = InferenceEngine::as<InferenceEngine::MemoryBlob>(dis_pred_blob);
            auto mdis_pred_holder = mdis_pred->rmap();
            const float *dis_pred = mdis_pred_holder.as<const float *>();

            auto mcls_pred = InferenceEngine::as<InferenceEngine::MemoryBlob>(cls_pred_blob);
            auto mcls_pred_holder = mcls_pred->rmap();
            const float *cls_pred = mcls_pred_holder.as<const float *>();
            this->decodeInfer(cls_pred, dis_pred, head_info.stride, score_threshold,results);
        }

        // std::vector<BoxInfo> dets;
        if (!box_result_vec_.empty())
            box_result_vec_.clear();

        for (int i = 0; i < (int)results.size(); i++)
        {
            this->nms(results[i]);

            for (auto &box : results[i])
            {
                box_result_vec_.push_back(box);
            }
        }
        std::sort(box_result_vec_.begin(), box_result_vec_.end(),
                  [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
        if (box_result_vec_.size() > 1)
            box_result_vec_.erase(box_result_vec_.begin() + 1, box_result_vec_.end());
    }

    void DeepExchanger::preProcess(cv::Mat &image, InferenceEngine::Blob::Ptr &blob)
    {
        int img_w = image.cols;
        int img_h = image.rows;
        int channels = 3;

        InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
        if (!mblob)
        {
            THROW_IE_EXCEPTION
                    << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
                    << "but by fact we were not able to cast inputBlob to MemoryBlob";
        }
        auto mblobHolder = mblob->wmap();
        float *blob_data = mblobHolder.as<float *>();

        for (size_t c = 0; c < channels; c++)
        {
            for (size_t h = 0; h < img_h; h++)
            {
                for (size_t w = 0; w < img_w; w++)
                {
                    blob_data[c * img_w * img_h + h * img_w + w] = (float)image.at<cv::Vec3b>(h, w)[c];
                }
            }
        }
    }

    void DeepExchanger::decodeInfer(const float *&cls_pred, const float *&dis_pred,
                               int stride, double threshold,
                               std::vector<std::vector<BoxInfo>> &results){
        int feature_h = ceil((float)image_size_ / stride);
        int feature_w = ceil((float)image_size_ / stride);
        for (int idx = 0; idx < feature_h * feature_w; idx++) {
            int row = idx / feature_w;
            int col = idx % feature_w;
            float score = 0;
            int cur_label = 0;
            for (int label = 0; label < num_class_; label++) {
                if (cls_pred[idx * num_class_ + label] > score) {
                    score = cls_pred[idx * num_class_ + label];
                    cur_label = label;
                }
            }
            if (score > threshold) {
                const float *bbox_pred = dis_pred + idx * 8;
                results[cur_label].push_back(
                        this->disPred2Bbox(bbox_pred, cur_label, score, col, row, stride));
            }
        }
    }

    BoxInfo DeepExchanger::disPred2Bbox(const float *&box_det, int label, double score, int x, int y, int stride)
    {
        float ct_x = (x + 0.5) * stride;
        float ct_y = (y + 0.5) * stride;
        //    float ct_x = (x + 0.5) * stride;
        //    float ct_y = (y + 0.5) * stride;
        std::vector<float> dis_pred;
        dis_pred.resize(8);
        for (int i = 0; i < 8; i++) {
            float dis = box_det[i];
            dis *= stride;
            dis_pred[i] = dis;
        }
        float x1 = (std::max)(ct_x + dis_pred[0], .0f);
        float y1 = (std::max)(ct_y + dis_pred[1], .0f);
        float x2 = (std::min)(ct_x + dis_pred[2], (float)this->image_size_);
        float y2 = (std::max)(ct_y + dis_pred[3], .0f);

        float x3 = (std::min)(ct_x + dis_pred[4], (float)this->image_size_);
        float y3 = (std::min)(ct_y + dis_pred[5], (float)this->image_size_);
        float x4 = (std::max)(ct_x + dis_pred[6], .0f);
        float y4 = (std::min)(ct_y + dis_pred[7], (float)this->image_size_);
        return BoxInfo{x1, y1, x2, y2, x3, y3, x4, y4, score, label};
    }

    void DeepExchanger::nms(std::vector<BoxInfo> &input_boxes)
    {
        //    for (auto it = input_boxes.begin(); it != input_boxes.end();)
        //    {
        //        if (it->label == 1 || it->label == 3)
        //        {
        //            it = input_boxes.erase(it);
        //        }
        //        else
        //        {
        //            ++it;
        //        }
        //    }

        std::sort(input_boxes.begin(), input_boxes.end(),
                  [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
        if (input_boxes.size() > 1)
            input_boxes.erase(input_boxes.begin() + 1, input_boxes.end());
    }


    void DeepExchanger::selectboxes(const cv::Mat &bgr,
                                  std::vector<BoxInfo> &bboxes)
    {
      cv::Mat match=cv::imread(ros::package::getPath("deep_exchanger")+"/IMG.jpg");
      cv::Mat trans(match.rows,match.cols,CV_8UC3);
      static int src_w = bgr.cols;
      static int src_h = bgr.rows;
      static float width_ratio = (float)src_w / (float)image_size_;
      static float height_ratio = (float)src_h / (float)image_size_;
      if(!bboxes.empty())
      {
        for(auto it=bboxes.begin();it!=bboxes.end();)
        {
          cv::Point2f src[3],dst[3];
          src[0]=cv::Point2f(it->x1 * width_ratio, it->y1 * height_ratio);
          src[1]=cv::Point2f(it->x2 * width_ratio, it->y2 * height_ratio);
          src[2]=cv::Point2f(it->x3 * width_ratio, it->y3 * height_ratio);
          dst[0]=cv::Point2f (match.cols-25,25);
          dst[1]=cv::Point2f (match.cols-25,match.rows-25);
          dst[2]=cv::Point2f (25,match.rows-25);

          double area = pointDist(cv::Point(it->x1,it->y1),cv::Point(it->x3,it->y3)) *
                        pointDist(cv::Point(it->x2,it->y2),cv::Point(it->x4,it->y4))/2;

          cv::Mat m=cv::getAffineTransform(src,dst);
          cv::warpAffine(bgr,trans,m,cv::Size(trans.cols,trans.rows),cv::INTER_CUBIC);

          cv::Mat result(trans.rows - match.rows + 1, trans.cols - match.cols + 1, CV_32FC1);
          cv::matchTemplate(trans,match,result,CV_TM_CCOEFF_NORMED);
          double dMaxVal;
          minMaxLoc(result, 0, &dMaxVal, 0, 0);
//          ROS_INFO("match=%0.2f",dMaxVal);

          if(dMaxVal < similarity_ || area < box_select_area_)
          {
            it = bboxes.erase(it);
          }
          else
          {
            ++it;
          }

//          sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", trans).toImageMsg();
//          trans_publisher_.publish(msg);
        }
      }
    }


    void DeepExchanger::drawBboxes(const cv::Mat &bgr,
                              const std::vector<BoxInfo> &bboxes)
    {
        std::string label_array[2] = {"Red", "Blue"};
        static int src_w = bgr.cols;
        static int src_h = bgr.rows;
        static float width_ratio = (float)src_w / (float)image_size_;
        static float height_ratio = (float)src_h / (float)image_size_;
        if (!bboxes.empty())
        {
            for (size_t i = 0; i < bboxes.size(); i++)
          {
                const BoxInfo &bbox = bboxes[i];
                cv::Point2f center(
                        (bbox.x1 + bbox.x2 + bbox.x3 + bbox.x4) / 4 * width_ratio,
                        (bbox.y1 + bbox.y2 + bbox.y3 + bbox.y4) / 4 * height_ratio);

                std::vector<cv::Point2f> points_vec;
                points_vec.emplace_back(center);
                points_vec.emplace_back(
                        cv::Point2f(bbox.x1 * width_ratio, bbox.y1 * height_ratio));
                points_vec.emplace_back(
                        cv::Point2f(bbox.x2 * width_ratio, bbox.y2 * height_ratio));
                points_vec.emplace_back(
                        cv::Point2f(bbox.x3 * width_ratio, bbox.y3 * height_ratio));
                points_vec.emplace_back(
                        cv::Point2f(bbox.x4 * width_ratio, bbox.y4 * height_ratio));

                static cv::Scalar color = cv::Scalar(205, 235, 255);
                cv::line(bgr, points_vec[1], points_vec[2], color, 1);
                cv::line(bgr, points_vec[2], points_vec[3], color, 1);
                cv::line(bgr, points_vec[3], points_vec[4], color, 1);
                cv::line(bgr, points_vec[4], points_vec[1], color, 1);
                for (int j = 0; j < 4 ; j++)
                    cv::putText(cv_image_->image,std::to_string(j + 1),points_vec[j + 1],1,3,cv::Scalar(0,255,0),1);

                cv::circle(bgr, points_vec[0], 3, color, cv::FILLED);
                //                        cv::putText(bgr,label_array[bbox.label],cv::Point2f(std::max(float(0),points_vec[1].x-10),std::max(points_vec[1].y-10,float(0))),cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,color,2);
                //                            cv::putText(bgr,std::to_string(bbox.score),cv::Point2f(std::max(float(0),points_vec[1].x-10),std::max(points_vec[1].y-10,float(0))),cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,color,2);


//                for(int i=0;i<imagePoints_.size();i++)
//                {
//                  cv::line(cv_image_->image,points_vec[0],cv::Point2f((imagePoints_[i].x/1440)*image_size_,(imagePoints_[i].y/1080)*image_size_),cv::Scalar(0,0,225),2);
//                }

            }
        }
    }

    inline void DeepExchanger::poseNonSensePnP()
    {
        rm_msgs::ExchangerMsg msg;
        msg = prev_msg_;
        msg.flag = 0;
        tf::Transform transform;

        transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y,
                                        msg.pose.position.z));
        transform.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                                             msg.pose.orientation.z, msg.pose.orientation.w));
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "exchanger"));

        pnp_publisher_.publish(msg);
    }

    void DeepExchanger::getPnP(cv::Mat &rvec, cv::Mat &tvec)
    {
        cv::Mat r_mat = cv::Mat_<double>(3, 3);

        if (rvec.empty())
        {
            ROS_INFO("opencv mat bug,return and pose nonsense pnp");
            poseNonSensePnP();
            return;
        }

        // the projection for the center of exchanger
        cv::Point3f point_o(0, 0, 0);
        cv::Point2f projected_point;
        std::vector<cv::Point3f> w_points_vector;
        std::vector<cv::Point2f> p_points_vector;
        w_points_vector.reserve(1);
        p_points_vector.reserve(1);
        w_points_vector.emplace_back(point_o);
        p_points_vector.emplace_back(projected_point);

        cv::projectPoints(w_points_vector, rvec, tvec, camera_matrix_, distortion_coefficients_, p_points_vector);
        if(is_show_center_)
            cv::circle(cv_image_->image, p_points_vector[0], 1, cv::Scalar(255, 255, 255), 1);

        cv::Rodrigues(rvec, r_mat);
        tf::Matrix3x3 tf_rotate_matrix(r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
                                       r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
                                       r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2));

        tf::Quaternion quaternion;
        double r;
        double p;
        double y;

        tf_rotate_matrix.getRPY(r, p, y);
        quaternion.setRPY(r,p,y);

        // upon for the origin pose and translation
        geometry_msgs::TransformStamped pose_in, pose_out;

        tf2::Quaternion tf_quaternion;
        //here for the tfListener
        tf_quaternion.setRPY(r,p,y);

        geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf_quaternion); // tmp for the pose

        if (!pose_signal_)
        {
            tf2::Transform cam2side, side2front, cam2front;
            cam2side.setOrigin(tf2::Vector3(tvec.at<double>(0,0),tvec.at<double>(0,1),tvec.at<double>(0,2)));
            cam2side.setRotation(tf2::Quaternion(quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w));
            if(direction_signal_)
            {
                side2front.setOrigin(tf2::Vector3(-0.1455,0,-0.144));
                side2front.setRotation(tf2::Quaternion(0,-1/sqrt((2)),0,1/sqrt((2))));
            }
            else
            {
                side2front.setOrigin(tf2::Vector3(0.1455,0,-0.144));
                side2front.setRotation(tf2::Quaternion(0,1/sqrt((2)),0,1/sqrt((2))));
            }

            cam2front = cam2side * side2front;
            pose_in.transform.translation.x = cam2front.getOrigin().x();
            pose_in.transform.translation.y = cam2front.getOrigin().y();
            pose_in.transform.translation.z = cam2front.getOrigin().z();

            pose_in.transform.rotation.x = cam2front.getRotation().x();
            pose_in.transform.rotation.y = cam2front.getRotation().y();
            pose_in.transform.rotation.z = cam2front.getRotation().z();
            pose_in.transform.rotation.w = cam2front.getRotation().w();
        }
        else
        {
            pose_in.transform.translation.x = tvec.at<double>(0,0);
            pose_in.transform.translation.y = tvec.at<double>(0,1);
            pose_in.transform.translation.z = tvec.at<double>(0,2);

            pose_in.transform.rotation.x = quat_msg.x;
            pose_in.transform.rotation.y = quat_msg.y;
            pose_in.transform.rotation.z = quat_msg.z;
            pose_in.transform.rotation.w = quat_msg.w;
        }

//        tf::Transform cam2front;
//        cam2front.setOrigin(tf::Vector3(pose_in.transform.translation.x,pose_in.transform.translation.y,pose_in.transform.translation.z));
//        cam2front.setRotation(tf::Quaternion(pose_in.transform.rotation.x, pose_in.transform.rotation.y,
//                                           pose_in.transform.rotation.z, pose_in.transform.rotation.w));
//        tf_broadcaster_.sendTransform(tf::StampedTransform(cam2front, ros::Time::now(), "camera_optical_frame", "exchanger"));

        try
        {
            tf2::doTransform(pose_in, pose_out, tf_buffer_.lookupTransform("map", "camera_optical_frame", ros::Time(0)));

        }
        catch (tf2::TransformException& ex)
        {
//        ROS_INFO_STREAM("tf error from ros");
//        ROS_WARN("%s", ex.what());
            return;
        }

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_out.transform.translation.x, pose_out.transform.translation.y,
                                        pose_out.transform.translation.z));
        transform.setRotation(tf::Quaternion(pose_out.transform.rotation.x, pose_out.transform.rotation.y,
                                             pose_out.transform.rotation.z, pose_out.transform.rotation.w));

        double roll_temp, pitch_temp, yaw_temp;
        quatToRPY(pose_out.transform.rotation, roll_temp, pitch_temp, yaw_temp);
        roll_temp +=CV_PI/2;
        yaw_temp +=CV_PI/2;

        tf2::Quaternion tmp_tf_quaternion;
        tmp_tf_quaternion.setRPY(-pitch_temp,-roll_temp,yaw_temp);
        geometry_msgs::Quaternion tmp_quat_msg = tf2::toMsg(tmp_tf_quaternion); // tmp for the pose
        transform.setRotation(tf::Quaternion(tmp_quat_msg.x, tmp_quat_msg.y,
                                             tmp_quat_msg.z, tmp_quat_msg.w));

        rm_msgs::ExchangerMsg msg;
        msg.flag = 1;
        if (shape_signal_) msg.shape = 1;
        else msg.shape = 0;

        msg.pose.position.x=pose_in.transform.translation.x;
        msg.pose.position.y=pose_in.transform.translation.y;
        msg.pose.position.z=pose_in.transform.translation.z;

        msg.pose.orientation.x=tmp_quat_msg.x;
        msg.pose.orientation.y=tmp_quat_msg.y;
        msg.pose.orientation.z=tmp_quat_msg.z;
        msg.pose.orientation.w=tmp_quat_msg.w;

        exchange_msg_.flag = msg.flag;
        exchange_msg_.pose = msg.pose;
        exchange_msg_.shape = msg.shape;
        pnp_publisher_.publish(exchange_msg_);

        double roll, pitch, yaw;
        quatToRPY(msg.pose.orientation, roll, pitch, yaw);
//    ROS_INFO_STREAM("X:       " << msg.pose.position.x);
//    ROS_INFO_STREAM("Y:       " << msg.pose.position.y);
//    ROS_INFO_STREAM("Z:       " << msg.pose.position.z);
//    ROS_INFO_STREAM("ROLL:    " << roll);
//    ROS_INFO_STREAM("PITCH:   " << pitch);
//    ROS_INFO_STREAM("YAW:     " << yaw);

        msg.pose.position.x=pose_out.transform.translation.x;
        msg.pose.position.y=pose_out.transform.translation.y;
        msg.pose.position.z=pose_out.transform.translation.z;
        prev_msg_ = msg;
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "exchanger"));

    }

    void DeepExchanger::quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
    {
        double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
        yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), square(q.w) + square(q.x) - square(q.y) - square(q.z));
        pitch = std::asin(as);
        roll = std::atan2(2 * (q.y * q.z + q.w * q.x), square(q.w) - square(q.x) - square(q.y) + square(q.z));
    }

    cv::Mat DeepExchanger::gammaTrans(cv::Mat &m_img, double gamma, int n_c)
    {
      cv::Mat m_imgGamma(m_img.size(), CV_32FC3);
      for (int i = 0; i < m_img.rows; i++)
      {
        for (int j = 0; j < m_img.cols; j++)
        {
          m_imgGamma.at<cv::Vec3f>(i, j)[0] = n_c * pow(m_img.at<cv::Vec3b>(i, j)[0], gamma);
          m_imgGamma.at<cv::Vec3f>(i, j)[1] = n_c * pow(m_img.at<cv::Vec3b>(i, j)[1], gamma);
          m_imgGamma.at<cv::Vec3f>(i, j)[2] = n_c * pow(m_img.at<cv::Vec3b>(i, j)[2], gamma);
        }
      }
      normalize(m_imgGamma, m_imgGamma, 0, 255, CV_MINMAX);
      convertScaleAbs(m_imgGamma, m_img);
      return m_img;
    }


    double DeepExchanger::square(double in)
    {
        double out = pow(in,2);
        return out;
    }

    void DeepExchanger::Lsd_LineDetect(cv::Mat image , std::vector<cv::Vec4f> &lines) {
      int width = image.cols;
      int height = image.rows;

      // 将图像数据转换为 LSD 所需的格式
      double* imageData = new double[width * height];
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          imageData[y * width + x] = static_cast<double>(image.at<uchar>(y, x));
        }
      }

      // 调用 LSD 检测直线段
      int n;
      int pInt = 1;
      double* output = lsd(&n, imageData, width, height, &pInt);

      // 输出检测结果
//      std::cout << "Detected " << n << " line segments:" << std::endl;
      for (int i = 0; i < n; ++i) {
        double x1 = output[i * 7 + 0];
        double y1 = output[i * 7 + 1];
        double x2 = output[i * 7 + 2];
        double y2 = output[i * 7 + 3];
        //        double width = output[i * 7 + 4];
        //        double p = output[i * 7 + 5];
        //        double log_nfa = output[i * 7 + 6];
        if(pointDist(cv::Point(x1,y1),cv::Point(x2,y2)) > 10)
        {
          if(x1 > x2)
          {
            lines.emplace_back(x2,y2,x1,y1);
          }
          else
          {
            lines.emplace_back(x1,y1,x2,y2);
          }
        }
      }

      // 释放内存
      delete[] imageData;
      if (output) {
        free(output);  // 释放 LSD 分配的内存
      }
    }

}