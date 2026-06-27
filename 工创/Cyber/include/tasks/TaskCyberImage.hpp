#ifndef TASKCYBERIMAGE_HPP_
#define TASKCYBERIMAGE_HPP_

#include <string.h>
#include <unistd.h>
#include <vector>
#include <mutex>

#include "SimOneServiceAPI.h"
#include "SimOneStreamingAPI.h"
#include "CyberWriterImage.hpp"

#include <thread>

typedef struct
{
    std::string ip = "127.0.0.1";
    unsigned short port = 13956;
} Img_Param_T;

class CW_Image
{
    public:
        CW_Image(std::string channel_name);
        ~CW_Image();
        void set_params(Img_Param_T& param);
        static void set_image(SimOne_Streaming_Image *pPointCloud);
        void task_image();
    private:
        CyberWriterImage mCyberImage;
        static CyberWriterImage* pCyberImage;
        Img_Param_T mParam;
};

#endif
