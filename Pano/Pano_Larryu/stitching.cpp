#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "utils.h"

#define ENABLE_LOG 1
#define LOG(msg) std::cout << msg
#define LOGLN(msg) std::cout << msg << std::endl


int main(int argc, char* argv[])
{
#if ENABLE_LOG
    int64 app_start_time = getTickCount();
#endif

#if 0
    cv::setBreakOnError(true);
#endif

    int retval = parseCmdArgs(argc, argv);
    if (retval)
        return retval;

    // Check if have enough images
    int num_images = static_cast<int>(img_names.size());
    if (num_images < 2)
    {
        // LOGLN("Need more images");
        return -1;
    }

    double work_scale = 1, seam_scale = 1, compose_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

#if ENABLE_LOG
    int64 t = getTickCount();
#endif

    Mat full_img, img;
    vector<Mat> images(num_images);
    vector<Size> full_img_sizes(num_images);
    double seam_work_aspect = 1;

    for (int i = 0; i < num_images; ++i)
    {
        full_img = imread(img_names[i]);
        full_img_sizes[i] = full_img.size();

        if (full_img.empty())
        {
            // LOGLN("Can't open image " << img_names[i]);
            return -1;
        }
        if (work_megapix < 0)
        {
            img = full_img;
            work_scale = 1;
            is_work_scale_set = true;
        }
        else
        {
            if (!is_work_scale_set)
            {
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), work_scale, work_scale);
            // cout << "First resize, scale: " << work_scale << endl;
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }

        resize(full_img, img, Size(), seam_scale, seam_scale);
        // cout << "Second resize, scale: " << seam_scale << endl;
        images[i] = img.clone();
    }


    // load indices
    vector<int> indices;
    FileStorage idx_fs("indices.xml", cv::FileStorage::READ);
    idx_fs["indices"] >> indices;
    // cout << "Loaded indices successfully." << endl;
    idx_fs.release();

    // Leave only images are from the same panorama
    vector<Mat> img_subset;
    vector<String> img_names_subset;
    vector<Size> full_img_sizes_subset;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        img_names_subset.push_back(img_names[indices[i]]);
        img_subset.push_back(images[indices[i]]);
        full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
    }

    images = img_subset;
    img_names = img_names_subset;
    full_img_sizes = full_img_sizes_subset;

    // Check if we still have enough images
    num_images = static_cast<int>(img_names.size());
    if (num_images < 2)
    {
        // LOGLN("Need more images");
        return -1;
    }

    vector<CameraParams> cameras(num_images);
    float warped_image_scale;

    // load camera parameters
    string xml_file_name = "camerasParameters.xml";
    loadCameraParams(xml_file_name, cameras, warped_image_scale);
    // LOGLN("Loaded cameras parameters successfully.");
    // cout << "Test CameraParams: print focal, focal = " << cameras[0].focal << endl; 


    // LOGLN("Warping images (auxiliary)... ");
#if ENABLE_LOG
    t = getTickCount();
#endif

    vector<Point> corners(num_images);
    vector<UMat> masks_warped(num_images);
    vector<UMat> images_warped(num_images);
    vector<Size> sizes(num_images);
    vector<UMat> masks(num_images);

    // Preapre images masks
    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    // Warp images and their masks

    Ptr<WarperCreator> warper_creator;
#ifdef HAVE_OPENCV_CUDAWARPING
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0) 
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarperGpu>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarperGpu>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarperGpu>();
    }
    else
#endif
    {
        if (warp_type == "plane")
            warper_creator = makePtr<cv::PlaneWarper>();
        else if (warp_type == "affine")
            warper_creator = makePtr<cv::AffineWarper>();
        else if (warp_type == "cylindrical")
            warper_creator = makePtr<cv::CylindricalWarper>();
        else if (warp_type == "spherical")
            warper_creator = makePtr<cv::SphericalWarper>();
        else if (warp_type == "fisheye")
            warper_creator = makePtr<cv::FisheyeWarper>();
        else if (warp_type == "stereographic")
            warper_creator = makePtr<cv::StereographicWarper>();
        else if (warp_type == "compressedPlaneA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
        else if (warp_type == "compressedPlaneA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
        else if (warp_type == "compressedPlanePortraitA2B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
        else if (warp_type == "compressedPlanePortraitA1.5B1")
            warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
        else if (warp_type == "paniniA2B1")
            warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
        else if (warp_type == "paniniA1.5B1")
            warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
        else if (warp_type == "paniniPortraitA2B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
        else if (warp_type == "paniniPortraitA1.5B1")
            warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
        else if (warp_type == "mercator")
            warper_creator = makePtr<cv::MercatorWarper>();
        else if (warp_type == "transverseMercator")
            warper_creator = makePtr<cv::TransverseMercatorWarper>();
    }

    if (!warper_creator)
    {
        // cout << "Can't create the following warper '" << warp_type << "'\n";
        return 1;
    }

    // Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));
    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale));
    // LOGLN("First warper scale: " << warped_image_scale);

    // cout << "success 0" << endl;

    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;

        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();

        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }

    // cout << "Success 1" << endl;

    vector<UMat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);

    // LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");


    // LOGLN("Compositing...");
#if ENABLE_LOG
    t = getTickCount();
#endif

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    Ptr<Timelapser> timelapser;
    //double compose_seam_aspect = 1;
    double compose_work_aspect = 1;

    /*----------------------v2 implementation, simplified version----------------------*/
    Size dst_sz = resultRoi(corners, sizes).size();
    float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;

    blender = Blender::createDefault(blend_type, try_cuda);
    MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
    // mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
    mb->setNumBands(0);
    // LOGLN("Multi-band blender, number of bands: " << mb->numBands());

    blender->prepare(corners, sizes);
    /*----------------------v2 implementation, simplified version----------------------*/

    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {
        // LOGLN("Compositing image #" << indices[img_idx]+1);

        /*----------------------v2 implementation, simplified version----------------------*/
        blender->feed(images_warped[img_idx], masks_warped[img_idx], corners[img_idx]);
        // print cout image size
        // if (img_idx == 0) cout << "images_warped #" << img_idx << " size " << images_warped[img_idx].size() << endl;
        /*----------------------v2 implementation, simplified version----------------------*/


    }

    if (!timelapse)
    {
        Mat result, result_mask;
        blender->blend(result, result_mask);

        // LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

        imwrite(result_name, result);
    }

    // LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
    double total_t = (getTickCount() - app_start_time) / getTickFrequency();
    double fps = 1.0 / total_t;
    LOGLN(fps << " fps");
    return 0;
}
