#include <SimOneStreamingAPI.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>
#include <string>
#include <queue>
#include <atomic>

std::queue<SimOne_Streaming_Image> messgeQueue;

extern "C"
{
#include "libswscale/swscale.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavformat/avformat.h"
#include "libavutil/opt.h"
}
#define WIDTH 1920
#define HEIGHT 1080
typedef unsigned char BYTE;

#pragma pack(1)
typedef struct RLE_Header {
	BYTE flag;
	char rle[3];
	unsigned short height;
	unsigned short width;
}Horizon_RLE_Header;
#pragma pack()

// std::string gIP = "127.0.0.1";
std::string gIP = "10.66.9.214";
unsigned short gPort = 13956;
unsigned short gPortRle = 13966;

SimOne_Streaming_Image gDataImage;
std::mutex	gDataImageMutex;


int Rle_Decode_Horizon(Horizon_RLE_Header &frameHeader, BYTE *inbuf, int inSize, BYTE *outbuf) {
	memcpy(&frameHeader, inbuf, sizeof(Horizon_RLE_Header));
	frameHeader.height = frameHeader.height >> 8 | frameHeader.height << 8;
	frameHeader.width = frameHeader.width >> 8 | frameHeader.width << 8;
	int outBuffSzie = frameHeader.width * frameHeader.height;

	BYTE *src = inbuf + sizeof(Horizon_RLE_Header);
	int i;
	int decSize = 0;
	while (src < (inbuf + inSize))
	{
		unsigned short count;
		memcpy(&count, src + 1, 2);

		if ((decSize + count) > outBuffSzie)
		{
			return -1;
		}
		BYTE value = *src;
		for (i = 0; i < count; i++)
		{
			outbuf[decSize++] = value;
		}
		src = src + 3;
	}
	return decSize;
}

std::string Buffer2Hex(unsigned char *buffer, int len) {
	std::string HEX = "0123456789ABCDEF";
	std::string finalHex = "";
	for (int i = 0; i < len; i++) {
		int high = buffer[i] >> 4;
		finalHex.push_back(HEX.at(high));
		int low = buffer[i] & 0x0F;
		finalHex.push_back(HEX.at(low));
	}
	return finalHex;
}
class HEVCDecoder {
public:
	const AVCodec *codec;
	AVCodecContext *c = nullptr;
	int frame_count;
	AVFrame *frame;
	AVFrame *pFrameBGR;

	int BGRsize;
	uint8_t *out_buffer = nullptr;

	struct SwsContext *img_convert_ctx;
	cv::Mat pCvMat;
	bool matReady;

public:
	void init() {
		matReady = false;
		avcodec_register_all();
		codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
		if (!codec) {
			fprintf(stderr, "Codec not found\n");
			exit(1);
		}
		c = avcodec_alloc_context3(codec);
		if (!c) {
			fprintf(stderr, "Could not allocate video codec context\n");
			exit(1);
		}

		if (avcodec_open2(c, codec, NULL) < 0) {
			fprintf(stderr, "Could not open codec\n");
			exit(1);
		}

		frame = av_frame_alloc();
		if (!frame) {
			fprintf(stderr, "Could not allocate video frame\n");
			exit(1);
		}

		frame_count = 0;
		//�洢�����ת����RGB����
		pFrameBGR = av_frame_alloc();
	}

	void decode(unsigned char *inputbuf, size_t size) {

		AVPacket avpkt;
		av_init_packet(&avpkt);
		memcpy(&avpkt.pts, inputbuf, 8);
		memcpy(&avpkt.dts, inputbuf + 8, 8);
		memcpy(&avpkt.stream_index, inputbuf + 16, 8);
		avpkt.size = size - 24;
		avpkt.data = inputbuf + 24;
		// std::string temp_str = Buffer2Hex(avpkt.data, 12);
		// std::cout << temp_str << std::endl;
		// if (temp_str == "0000000140010C01FFFF0160") {
		// 	std::cout << "11111111111111111111111111111111111111111" << std::endl;
		// }
		// else {
		// 	std::cout << "00000000000000000000000000000000000000000" << std::endl;
		// }

		int len, got_frame;
		len = avcodec_decode_video2(c, frame, &got_frame, &avpkt);
		av_packet_unref(&avpkt);
		if (len < 0) {
			matReady = false;
			fprintf(stderr, "Error while decoding frame %d\n", frame_count);
			frame_count++;

			return;
		}
		if (out_buffer == nullptr) {
			BGRsize = avpicture_get_size(AV_PIX_FMT_BGR24, c->width,
				c->height);
			out_buffer = (uint8_t *)av_malloc(BGRsize);
			avpicture_fill((AVPicture *)pFrameBGR, out_buffer, AV_PIX_FMT_BGR24,
				c->width, c->height);

			img_convert_ctx =
				sws_getContext(c->width, c->height, c->pix_fmt,
					c->width, c->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL,
					NULL);
			pCvMat.create(cv::Size(c->width, c->height), CV_8UC3);

		}
		if (got_frame) {
			matReady = true;
			sws_scale(img_convert_ctx, (const uint8_t *const *)frame->data,
				frame->linesize, 0, c->height, pFrameBGR->data, pFrameBGR->linesize);

			memcpy(pCvMat.data, out_buffer, BGRsize);

			//        printf("decoding frame: %d\n",frame_count);
			frame_count++;
			av_frame_unref(frame);
		}
		else {
			matReady = false;
		}
	}

	void play() {
		if (matReady) {
			cv::imshow("51Sim-One Camera Video Injection H265", pCvMat);
		}
	}

	HEVCDecoder() {
		init();
	}

	cv::Mat getMat() {
		if (matReady) {
			return pCvMat;
		}
		else {
			return cv::Mat();
		}
	}
};

class HEVCEncoder {
private:
	bool mInited;
	int mWidth;
	int mHeight;
	AVCodecContext *mpFFCodecCtx = nullptr;
	AVCodecContext *mpFFCodec2H265 = nullptr;
	AVFormatContext* fc = nullptr;
	AVStream* stream = nullptr;
	AVFrame *mpFFFrameYuv = nullptr;
	AVPacket *mpFFPackage = nullptr;
	AVPacket pkt;
	SwsContext *mpFFConvertCtx = nullptr;
	AVBufferRef *hw_device_ctx = NULL;

public:
	int set_hwframe_ctx(AVCodecContext *ctx, AVBufferRef *hw_device_ctx)
	{
		AVBufferRef *hw_frames_ref;
		AVHWFramesContext *frames_ctx = NULL;
		int err = 0;

		if (!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx))) {
			//fprintf(stderr, "Failed to create CUDA frame context.\n");
			return -1;
		}
		frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
		frames_ctx->format = AV_PIX_FMT_CUDA;
		frames_ctx->sw_format = AV_PIX_FMT_YUV420P;
		frames_ctx->width = ctx->width;
		frames_ctx->height = ctx->height;
		frames_ctx->initial_pool_size = 20;
		if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
			//fprintf(stderr, "Failed to initialize CUDA frame context."
				  //  "Error code: %s\n",av_err2str(err));
			av_buffer_unref(&hw_frames_ref);
			return err;
		}
		ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
		if (!ctx->hw_frames_ctx)
			err = AVERROR(ENOMEM);

		av_buffer_unref(&hw_frames_ref);
		return err;
	}

	int hw_encoder_init(AVCodecContext *ctx, const enum AVHWDeviceType type)
	{
		int err = 0;

		if ((err = av_hwdevice_ctx_create(&hw_device_ctx, type, NULL, NULL, 0)) < 0)
		{
			printf("Failed to create specified Cuda HW device.\n");
			return err;
		}
		ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);

		return err;
	}

	bool initFFmpeg2H265(int width, int height)
	{
		// Setting up the codec.
		int ret;
		// jpeg ---------------------------yuv 420 p
		av_register_all();
		avcodec_register_all();

		AVCodec *pCodec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
		if (!pCodec)
		{
			printf("avcodec_find_decoder failed");
			return false;
		}
		mpFFCodecCtx = avcodec_alloc_context3(pCodec);
		mpFFCodecCtx->pix_fmt = AV_PIX_FMT_CUDA;
		mpFFCodecCtx->width = width;
		mpFFCodecCtx->height = height;
		mpFFCodecCtx->time_base.num = 1;
		mpFFCodecCtx->time_base.den = 60;
		mpFFCodecCtx->sample_aspect_ratio.num = 1;
		mpFFCodecCtx->sample_aspect_ratio.den = 1;

		if (!mpFFCodecCtx)
		{
			printf("avcodec_alloc_context3 failed");
			return false;
		}
		if (avcodec_open2(mpFFCodecCtx, pCodec, NULL) < 0)
		{
			printf("avcodec_open2 failed");
			return false;
		}

		// yuv420p-------->h265

		AVCodec *encoder = avcodec_find_encoder_by_name("hevc_nvenc");
		// encoder = avcodec_find_encoder(dec_ctx->codec_id);
		if (!encoder)
		{
			av_log(NULL, AV_LOG_FATAL, "Necessary encoder not found\n");
			return AVERROR_INVALIDDATA;
		}
		mpFFCodec2H265 = avcodec_alloc_context3(encoder);
		if (!mpFFCodec2H265)
		{
			av_log(NULL, AV_LOG_FATAL, "Failed to allocate the encoder context\n");
			return AVERROR(ENOMEM);
		}
		mpFFCodec2H265->width = width;
		mpFFCodec2H265->height = height;
		// enc_ctx->sample_aspect_ratio = dec_ctx->sample_aspect_ratio;
		/* take first format from list of supported formats */
		mpFFCodec2H265->pix_fmt = AV_PIX_FMT_CUDA; // encoder->pix_fmts[0];
		// mpFFCodec2H265->pix_fmt = AV_PIX_FMT_YUV420P;
		int fps = 30;
		int bit_rate_ = 8192000;
		mpFFCodec2H265->time_base = AVRational{ 1, fps };
		mpFFCodec2H265->bit_rate = bit_rate_; // 8192000
		mpFFCodec2H265->keyint_min = fps;
		mpFFCodec2H265->max_b_frames = 0;
		mpFFCodec2H265->gop_size = fps;

		/* video time_base can be set to whatever is handy and supported by encoder */
		mpFFCodec2H265->time_base = AVRational{ 1, fps };
		if (hw_encoder_init(mpFFCodec2H265, AV_HWDEVICE_TYPE_CUDA) < 0)
			return -1;
		if (set_hwframe_ctx(mpFFCodec2H265, hw_device_ctx) < 0)
		{
			fprintf(stderr, "Failed to set hwframe context.\n");
			return -1;
		}
		// int ret =0;
		AVDictionary *param = NULL;
		av_dict_set(&param, "preset", "slow", 0);
		av_dict_set(&param, "tune", "zero-latency", 0);
		if ((ret = avcodec_open2(mpFFCodec2H265, encoder, &param)) < 0)
		{
			printf("Failed to open codec for stream \n");
			return -1;
		}

		mpFFFrameYuv = av_frame_alloc();
		if (!mpFFFrameYuv)
		{
			printf("av_frame_alloc mpFFFrameYuv failed");
			return false;
		}
		mpFFPackage = new AVPacket();

		return true;
	}

	int encodeToH265Image(int width, int height, int inSize, const char *inImage, char *&outImage)
	{
		AVFrame *frame = NULL;
		AVFrame *hw_frame = NULL;
		AVPacket packet; //= { .data = NULL, .size = 0 };
		av_init_packet(&packet);
		packet.size = inSize;
		packet.data = (uint8_t *)inImage;
		AVPacket pkt;
		av_init_packet(&pkt);
		pkt.data = NULL;
		pkt.size = 0;
		int got_frame = 0;
		if (!mInited || width != mWidth || height != mHeight)
		{
			mWidth = width;
			mHeight = height;
			printf("Init image decoder with size(%d, %d)\n", mWidth, mHeight);
			freeFFmpeg();
			mInited = initFFmpeg2H265(width, height);
		}
		av_log(NULL, AV_LOG_DEBUG, "Going to reencode&filter the frame\n");
		frame = av_frame_alloc();
		if (!frame)
		{
			printf("alloc frame fail");
			return -1;
		}
		int ret = avcodec_decode_video2(mpFFCodecCtx, frame,
			&got_frame, &packet);
		if (ret < 0)
		{
			av_frame_free(&frame);
			av_log(NULL, AV_LOG_ERROR, "Decoding failed\n");
			return -1;
		}
		// else{
		// 	printf("Decoding pkt_size=%d, pkt_pos=%d ,format=%d\n",
		// 	frame->pkt_size,
		// 	frame->pkt_pos,
		// 	frame->format);
		// }

		if (got_frame)
		{
			frame->pts = frame->best_effort_timestamp;
			/* read data into software frame, and transfer them into hw frame */
			if (!(hw_frame = av_frame_alloc()))
			{
				return -1;
			}
			if (av_hwframe_get_buffer(mpFFCodec2H265->hw_frames_ctx, hw_frame, 0) < 0)
			{
				printf("av_hwframe_get_buffer");
				return -1;
			}
			if (!hw_frame->hw_frames_ctx)
			{
				printf("hw_frame");
				return -1;
			}
			if (av_hwframe_transfer_data(hw_frame, frame, 0) < 0)
			{
				printf("Error while transferring frame data to surface.");
				return -1;
			}
			got_frame = 0;
			// ret = filter_encode_write_frame(hw_frame, 0,&pkt,&got_frame);
			ret = avcodec_encode_video2(mpFFCodec2H265, &pkt, hw_frame, &got_frame);
			av_frame_free(&hw_frame);
			if ((ret < 0) || (got_frame < 1))
				return -1;
		}
		else
		{
			av_frame_free(&frame);
		}

		int64_t pts = pkt.pts;
		int64_t dts = pkt.dts;
		int64_t stream_index = pkt.stream_index;

		unsigned char *avpacket_current = (uint8_t *)malloc(sizeof(unsigned char) * (24 + pkt.size));
		memset(avpacket_current, 0, sizeof(uint8_t) * 3 + pkt.size);
		memcpy(avpacket_current, &pts, sizeof(int64_t));
		memcpy(avpacket_current + sizeof(int64_t), &dts, sizeof(int64_t));
		memcpy(avpacket_current + 2 * sizeof(int64_t), &stream_index, sizeof(int64_t));
		memcpy(avpacket_current + 3 * sizeof(int64_t), pkt.data, pkt.size);

		size_t nStreamSize = pkt.size + 3 * sizeof(int64_t);
		outImage = (char *)(avpacket_current);
		av_packet_unref(&pkt);
		return nStreamSize;
	}

	void freeFFmpeg()
	{
		if (mpFFCodecCtx)
		{
			avcodec_close(mpFFCodecCtx);
			mpFFCodecCtx = nullptr;
		}
		if (mpFFCodec2H265)
		{
			avcodec_close(mpFFCodec2H265);
			mpFFCodecCtx = nullptr;
		}
		if (mpFFConvertCtx)
		{
			sws_freeContext(mpFFConvertCtx);
			mpFFConvertCtx = nullptr;
		}
		if (mpFFFrameYuv)
		{
			av_frame_free(&mpFFFrameYuv);
			mpFFFrameYuv = nullptr;
		}
		if (mpFFPackage)
		{
			av_free_packet(mpFFPackage);
			mpFFPackage = nullptr;
		}
	}
};

void dataImageCallback(SimOne_Streaming_Image *pImage)
{
	//std::lock_guard<std::mutex> lock(gDataImageMutex);
	gDataImage.frame = pImage->frame;
	gDataImage.timestamp = pImage->timestamp;
	gDataImage.height = pImage->height;
	gDataImage.width = pImage->width;
	gDataImage.imageDataSize = pImage->imageDataSize;
	gDataImage.format = pImage->format;
	memcpy(&gDataImage.imageData, &pImage->imageData, pImage->imageDataSize);
	messgeQueue.push(gDataImage);
}


int main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		gIP = argv[1];
		gPort = atoi(&*argv[2]);
	}

	std::thread mImageProcess([&]() {
		int lastFrame = 0;
		HEVCEncoder encoder;
		HEVCDecoder decoder;
		while (1)
		{
			std::unique_ptr<SimOne_Streaming_Image> gDataImage = std::make_unique<SimOne_Streaming_Image>();
			if (SimOneAPI::GetStreamingImage(gIP.c_str(), gPort, gDataImage.get())) {
				if (gDataImage->frame != lastFrame) {
					if (gDataImage->format == ESimOne_Streaming_Image_Format_RGB) {
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::imshow("51Sim-One Camera Video Injection Rgb", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_RLESegmentation) {
						Horizon_RLE_Header frameHeader;
						char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
						Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImage->imageData), gDataImage->imageDataSize, (BYTE*)(void*)ImageRleData);
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8U, ImageRleData);
						cv::imshow("51Sim-One Camera Video Injection Rle", img);
						delete ImageRleData;
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_JPEG) {
						cv::Mat imgbuf(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
						cv::imshow("51Sim-One Camera Video Injection Jpeg", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_H265) {
						char *imageTemp = nullptr;
						int sizeofhevc = encoder.encodeToH265Image(gDataImage->width, gDataImage->height, gDataImage->imageDataSize, gDataImage->imageData, imageTemp);
						 if(sizeofhevc>0){
						 	printf("messgeQueue's size = %d ,encode success and size = %d\n",messgeQueue.size(),sizeofhevc);
						 }else{
						 	printf("fail !!!!!!!!!!!!!!\n");
						 }
						if (sizeofhevc > 0) {
							decoder.decode((unsigned char *)imageTemp, sizeofhevc);
							decoder.play();
							delete(imageTemp);
						}
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
					//std::this_thread::sleep_for(std::chrono::nanoseconds(1));
					if (cv::waitKey(1) == 27)
						break;
					lastFrame = gDataImage->frame;
				}
			}
		}});

	std::thread mImageProcessRle([&]() {
		int lastFrame = 0;
		HEVCEncoder encoder;
		HEVCDecoder decoder;
		while (1)
		{
			std::unique_ptr<SimOne_Streaming_Image> gDataImage = std::make_unique<SimOne_Streaming_Image>();
			if (SimOneAPI::GetStreamingImage(gIP.c_str(), gPortRle, gDataImage.get())) {
				if (gDataImage->frame != lastFrame) {
					if (gDataImage->format == ESimOne_Streaming_Image_Format_RGB) {
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::imshow("51Sim-One Camera Video Injection Rgb", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_RLESegmentation) {
						Horizon_RLE_Header frameHeader;
						char * ImageRleData = new char[SOSM_IMAGE_DATA_SIZE_MAX];
						Rle_Decode_Horizon(frameHeader, (BYTE*)(void*)&(gDataImage->imageData), gDataImage->imageDataSize, (BYTE*)(void*)ImageRleData);
						cv::Mat img(gDataImage->height, gDataImage->width, CV_8U, ImageRleData);
						cv::imshow("51Sim-One Camera Video Injection Rle", img);
						delete ImageRleData;
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_JPEG) {
						cv::Mat imgbuf(gDataImage->height, gDataImage->width, CV_8UC3, gDataImage->imageData);
						cv::Mat img = imdecode(imgbuf, CV_LOAD_IMAGE_COLOR);
						cv::imshow("51Sim-One Camera Video Injection Jpeg", img);
					}
					else if (gDataImage->format == ESimOne_Streaming_Image_Format_H265) {
						char *imageTemp = nullptr;
						int sizeofhevc = encoder.encodeToH265Image(gDataImage->width, gDataImage->height, gDataImage->imageDataSize, gDataImage->imageData, imageTemp);
						if (sizeofhevc > 0) {
							printf("messgeQueue's size = %d ,encode success and size = %d\n", messgeQueue.size(), sizeofhevc);
						}
						else {
							printf("fail !!!!!!!!!!!!!!\n");
						}
						if (sizeofhevc > 0) {
							decoder.decode((unsigned char *)imageTemp, sizeofhevc);
							decoder.play();
							delete(imageTemp);
						}
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
					//std::this_thread::sleep_for(std::chrono::nanoseconds(1));
					if (cv::waitKey(1) == 27)
						break;
					lastFrame = gDataImage->frame;
				}
			}
		}});

	mImageProcess.detach();
	mImageProcessRle.detach();
	while (1) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	return 0;
}
