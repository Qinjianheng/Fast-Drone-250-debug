#include "stdio.h"
extern "C"{
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

#include <opencv2/opencv.hpp>
#include <string>

class AprilTagDetector
{
private:
    const std::string family;
    apriltag_family_t* tf;
    apriltag_detector_t* td;
public:
    AprilTagDetector(const std::string& family_): family(family_)
    {
        if (family == "tag36h11")
            tf = tag36h11_create();
        else if (family == "tag25h9")
            tf = tag25h9_create();
        else if (family == "tag16h5")
            tf = tag16h5_create();
        else if (family == "tagCircle21h7")
            tf = tagCircle21h7_create();
        else if (family == "tagCircle49h12")
            tf = tagCircle49h12_create();
        else if (family == "tagStandard41h12")
            tf = tagStandard41h12_create();
        else if (family == "tagStandard52h13")
            tf = tagStandard52h13_create();
        else if (family == "tagCustom48h12")
            tf = tagCustom48h12_create();
        else{
            printf("Unrecognized tag family name!\n");
            exit(-1);
        }

        td = apriltag_detector_create();
        apriltag_detector_add_family_bits(td, tf, 1);

        switch(errno){
            case EINVAL:
                printf("\"hamming\" parameter is out-of-range.\n");
                exit(-1);
            case ENOMEM:
                printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder\n");
                exit(-1);
        }

        td->quad_decimate = 2.f;
        td->quad_sigma = 0.f;
        td->nthreads = 1;
        td->debug = false;
        td->refine_edges = true;
    }

    ~AprilTagDetector()
    {
        apriltag_detector_destroy(td);
        if (family == "tag36h11")
            tag36h11_destroy(tf);
        else if (family == "tag25h9")
            tag25h9_destroy(tf);
        else if (family == "tag16h5")
            tag16h5_destroy(tf);
        else if (family == "tagCircle21h7")
            tagCircle21h7_destroy(tf);
        else if (family == "tagCircle49h12")
            tagCircle49h12_destroy(tf);
        else if (family == "tagStandard41h12")
            tagStandard41h12_destroy(tf);
        else if (family == "tagStandard52h13")
            tagStandard52h13_destroy(tf);
        else if (family == "tagCustom48h12")
            tagCustom48h12_destroy(tf);
    }

    void detect(cv::Mat& img)
    {
        cv::Mat gray; cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };
        zarray_t *detections = apriltag_detector_detect(td, &im);
        if (errno == EAGAIN) {
            printf("Unable to create the %d threads requested.", td->nthreads);
            exit(-1);
        }

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(img, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Scalar(0, 0xff, 0), 2);
            line(img, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0, 0, 0xff), 2);
            line(img, cv::Point(det->p[1][0], det->p[1][1]),
                     cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Scalar(0xff, 0, 0), 2);
            line(img, cv::Point(det->p[2][0], det->p[2][1]),
                     cv::Point(det->p[3][0], det->p[3][1]),
                     cv::Scalar(0xff, 0, 0), 2);

            std::string id = std::to_string(det->id);
            int baseline;
            cv::Size textsize = getTextSize(id, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, 2, &baseline);
            putText(img, id, cv::Point(det->c[0] - (double)textsize.width / 2, det->c[1] + (double)textsize.height / 2),
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, cv::Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);
    }
};