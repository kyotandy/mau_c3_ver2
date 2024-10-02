#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp> // OpenCV for image processing
#include "MvCameraControl.h"

// PressEnterToExit function: waits for user to press enter to exit
void PressEnterToExit(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n');
}

std::queue<cv::Mat> imageQueue; // Queue to hold images
std::mutex mtx; // Mutex for thread-safe access to the queue
std::condition_variable conditionVar; // Condition variable for signaling
bool isRunning = true; // Flag to control the running state of threads

// Image callback function
void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) {
    if (pData == nullptr || pFrameInfo == nullptr) {
        std::cerr << "Invalid data received in callback!" << std::endl;
        return;
    }

    if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerBG8) {
        // Create a cv::Mat with BayerBG8 data
        cv::Mat bayerImage(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);

        // Convert the Bayer image to RGB
        cv::Mat rgbImage;
        cv::cvtColor(bayerImage, rgbImage, cv::COLOR_BayerBG2RGB);

        if (!rgbImage.empty()) {
            // Lock the queue and push the image
            {
                std::lock_guard<std::mutex> lock(mtx);
                imageQueue.push(rgbImage.clone());
            }

            // Notify the display thread that a new image is available
            conditionVar.notify_one();
        } else {
            std::cerr << "Failed to convert Bayer to RGB!" << std::endl;
        }
    } else {
        std::cerr << "Unsupported pixel format!" << std::endl;
    }
}



// Thread to process and publish images from the queue
void publishImages(image_transport::Publisher& publisher) {
    while (isRunning) {
        std::unique_lock<std::mutex> lock(mtx);
        conditionVar.wait(lock, [] { return !imageQueue.empty() || !isRunning; }); // Wait until there's data in the queue

        if (!isRunning && imageQueue.empty()) {
            break; // Exit if the program is stopped and no images are left
        }

        // Get the image from the queue
        cv::Mat image = imageQueue.front();
        imageQueue.pop();

        lock.unlock();

        // Publish the image using ROS
        if (!image.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            publisher.publish(msg);
        }
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    // Create an image_transport instance and publisher for ROS images
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imagePub = it.advertise("camera/image_raw", 10);

    int nRet = MV_OK;
    void* handle = nullptr;

    // Initialize the camera
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
        std::cerr << "No camera found!" << std::endl;
        return -1;
    }

    // Select and open the camera
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet) {
        std::cerr << "Failed to create handle!" << std::endl;
        return -1;
    }
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        std::cerr << "Failed to open device!" << std::endl;
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    // Set trigger mode to off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
        std::cerr << "Failed to set trigger mode!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    // Set image size to 1000x1000
    nRet = MV_CC_SetIntValue(handle, "Width", 1000);
    if (MV_OK != nRet) {
        std::cerr << "Failed to set Width!" << std::endl;
    }

    nRet = MV_CC_SetIntValue(handle, "Height", 1000);
    if (MV_OK != nRet) {
        std::cerr << "Failed to set Height!" << std::endl;
    }

    nRet = MV_CC_SetIntValue(handle, "OffsetY", 580);
    if (MV_OK != nRet) {
        std::cerr << "Failed to set OffsetY!" << std::endl;
    }

    nRet = MV_CC_SetIntValue(handle, "OffsetX", 800);
    if (MV_OK != nRet) {
        std::cerr << "Failed to set OffsetX!" << std::endl;
    }

    MVCC_ENUMVALUE stPixelFormat = {0};
    nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &stPixelFormat);
    if (MV_OK == nRet) {
        std::cout << "Current Pixel Format: " << stPixelFormat.nCurValue << std::endl;
    } else {
        std::cerr << "Failed to get pixel format! nRet = [" << nRet << "]" << std::endl;
    }

    // Register the callback function
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (MV_OK != nRet) {
        std::cerr << "Failed to register callback!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    // Start grabbing images
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        std::cerr << "Failed to start grabbing!" << std::endl;
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        return -1;
    }

    // Start the publish thread
    std::thread publishThread(publishImages, std::ref(imagePub));

    // Wait for user input to exit
    PressEnterToExit();
    isRunning = false;

    // Stop grabbing images
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);

    // Notify the publish thread to exit
    conditionVar.notify_one();
    publishThread.join();

    return 0;
}
