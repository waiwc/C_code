// 将HttpServer.h放到httplib.h文件的同级目录下, 就能按照文章中提到的方式使用它们了
#pragma once
#include "httplib.h"
#include <functional>
#include <iostream>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <vector>


/**
 * @brief 该宏为1时, 打开AutoBashController的代码. 为0时, 关闭AutoBashController的代码
 *
 * 如果编译不过去或绑定失败, 将该值改为0. 请只使用BashController(AutoBashController只是更方便一点, 功能并无不同)
 */
#define HTTPSERVER_AUTO_CONTROLLER 1


namespace httplib {
	// 线程池(复制的httplib的线程池, 添加了动态增加线程数量的功能)
	class MyHttpServerThreadPool {
	public:
		explicit MyHttpServerThreadPool(size_t n);

		void addAThread();

		std::size_t getThreadNum();

		MyHttpServerThreadPool(const ThreadPool&) = delete;
		~MyHttpServerThreadPool();

		void enqueue(std::function<void()> fn);

		void shutdown();

	private:
		struct worker {
			explicit worker(MyHttpServerThreadPool& pool) : pool_(pool) {}
			void operator()();
			MyHttpServerThreadPool& pool_;
		};
		friend struct worker;

		std::vector<std::thread> threads_;
		std::list<std::function<void()>> jobs_;

		bool shutdown_;

		std::condition_variable cond_;
		std::mutex mutex_;
	};

	class BaseController;
#if HTTPSERVER_AUTO_CONTROLLER
	class AutoBashController;
#endif
	// HttpServer, 用于管理httplib::Server对象
	class HttpServer {
	private:
		std::string host = "";
		int port = -1;
		static httplib::MyHttpServerThreadPool* serverPool;
		httplib::Server* server = new httplib::Server();
		static std::atomic<int> poolUseNum;
		int socket_flags;
		void buildServerThreadPool();
		friend class BaseController;
		std::vector<BaseController*> controllerVec;
		httplib::Server::Handler default_error_handler = [](const httplib::Request& req, httplib::Response& resp) {
			std::string error_code = std::to_string(resp.status);
			resp.set_content(error_code, "text/plain");
		};
#if HTTPSERVER_AUTO_CONTROLLER
		friend class AutoBashController;
		std::vector<AutoBashController*> autoControllerVec;

#endif

	public:
		HttpServer(const HttpServer&) = delete;
		HttpServer() = delete;

		HttpServer(const std::string& _host, int _port, int _socket_flags = 0);

		HttpServer(int _port, int _socket_flags = 0);
		void addController(httplib::BaseController* controller);
#if HTTPSERVER_AUTO_CONTROLLER
		void addController(httplib::AutoBashController* controller);
#endif
		httplib::Server* getHttplibServer();
		/*
			如果serverPool为null, 将为static serverPool 创建一个新的HttpThreadPool, 并在该线程池中监听
			如果serverPool不为null, 将直接使用static serverPool, 在线程池中执行httplib::Server的listen函数
		*/
		void listenInThread();

		// 在本地监听httplib::Server的listen函数
		void listenInLocal();

		// 释放server指针,如果poolUseNum为0, 也将释放serverPool
		~HttpServer();
	};

	// BashController, 模仿java spring mvc的开发风格
	class BaseController {
	public:
		BaseController() = default;
		BaseController(BaseController&) = delete;


		~BaseController();

	protected:
		// 必须重写bind方法, 在其中绑定具体的请求响应地址和请求响应方法, 否则抛出一个string异常
		virtual void bind();
		httplib::Server* server = nullptr;
		friend class httplib::HttpServer;
		// 绑定函数
		template <class Func, class T>
		auto BindController(Func&& func, T&& obj)
		{
			httplib::Server::Handler handler = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
			return handler;
		}
		void replace_all(std::string& str, const std::string& old_value, const std::string& new_value);

	private:
		void initToServer(httplib::Server* _server);
		void initToServer(httplib::HttpServer* _server);
	};

}   // namespace httplib


#if HTTPSERVER_AUTO_CONTROLLER
namespace httplib {
	enum class HttpServerRequestType { GET, POST, PUT, PATCH, DEL, Options };
	enum class HttpServerFilterType { REQUEST_FILTER, RESPONCE_FILTER };


	class AutoBashController {
		friend httplib::HttpServer;
		static std::unordered_map<std::string, std::pair<HttpServerRequestType, httplib::Server::Handler>> __autoBindFuncMap;
		static std::mutex __autoMapResetMtx;

	public:
		void replace_all(std::string& str, const std::string& old_value, const std::string& new_value);

	protected:
		// 当这个结构体初始化时, 会把绑定函数放到map里面
		struct PathStruct__;
		struct FilterStruct__;

		httplib::Server::Handler __request_filter = nullptr;
		httplib::Server::Handler __response_filter = nullptr;

	private:
		httplib::Server* __httplib_server;
		virtual void bind();
		void initToServer(httplib::Server* _server);
		void initToServer(httplib::HttpServer* _server);
	};
}   // namespace httplib

#endif
/* ============================================================== */
/* ============================================================== */
/* =========================== 源文件 ============================ */
/* ============================================================== */
/* ============================================================== */

// #include "HttpServer.h"


inline httplib::MyHttpServerThreadPool::MyHttpServerThreadPool(size_t n) : shutdown_(false)
{
	while (n)
	{
		std::thread task(worker(*this));
		task.detach();
		threads_.emplace_back(std::move(task));

		n--;
	}
}

inline void httplib::MyHttpServerThreadPool::addAThread()
{
	std::thread task(worker(*this));
	task.detach();
	threads_.emplace_back(std::move(task));
}

inline std::size_t httplib::MyHttpServerThreadPool::getThreadNum()
{
	return threads_.size();
}

inline httplib::MyHttpServerThreadPool::~MyHttpServerThreadPool()
{
	shutdown();
}
inline void httplib::MyHttpServerThreadPool::enqueue(std::function<void()> fn)
{
	{
		std::unique_lock<std::mutex> lock(mutex_);
		jobs_.push_back(std::move(fn));
	}

	cond_.notify_one();
}
inline void httplib::MyHttpServerThreadPool::shutdown()
{
	// Stop all worker threads...
	{
		std::unique_lock<std::mutex> lock(mutex_);
		shutdown_ = true;
	}

	// 停止任务添加器
	cond_.notify_all();
}

inline void httplib::MyHttpServerThreadPool::worker::operator()()
{
	for (;;)
	{
		std::function<void()> fn;
		{
			std::unique_lock<std::mutex> lock(pool_.mutex_);

			pool_.cond_.wait(lock, [&] {
				return !pool_.jobs_.empty() || pool_.shutdown_;
			});

			if (pool_.shutdown_ && pool_.jobs_.empty())
			{
				break;
			}

			fn = std::move(pool_.jobs_.front());
			pool_.jobs_.pop_front();
		}

		assert(true == static_cast<bool>(fn));
		fn();
	}
}



inline httplib::HttpServer::HttpServer(const std::string& _host, int _port, int _socket_flags)
	: host(std::move(_host)), port(_port), socket_flags(_socket_flags)
{
	server->set_error_handler(this->default_error_handler);
}


inline httplib::HttpServer::HttpServer(int _port, int _socket_flags)
	: host(std::move("0.0.0.0")), port(_port), socket_flags(_socket_flags)
{
	server->set_error_handler(this->default_error_handler);
}

inline void httplib::HttpServer::buildServerThreadPool()
{
	poolUseNum++;

	if (serverPool == nullptr)
	{
		// 创建了一个空线程池, 里面没有线程
		serverPool = new httplib::MyHttpServerThreadPool{ 0 };
	}
}



inline void httplib::HttpServer::addController(httplib::BaseController* controller)
{
	controllerVec.push_back(controller);
	controller->initToServer(this);
}



inline httplib::Server* httplib::HttpServer::getHttplibServer()
{
	return server;
}

inline void httplib::HttpServer::listenInThread()
{

	buildServerThreadPool();

	if (serverPool != nullptr)
	{
		serverPool->addAThread();
		std::cout << "listen to " << port << std::endl;
		serverPool->enqueue(std::move(std::bind(&httplib::Server::listen, server, host, port, socket_flags)));
		return;
	}

	std::cout << __func__ << " failed, httplib::HttpServer: threadpool is nullptr" << std::endl;
}

inline void httplib::HttpServer::listenInLocal()
{
	bool ret = server->listen(host, port, socket_flags);
}

inline httplib::HttpServer::~HttpServer()
{
	poolUseNum--;

	if (this->server != nullptr)
	{
		server->stop();
		delete server;
	}

	if (poolUseNum == 0 && serverPool != nullptr)
	{
		delete serverPool;
		serverPool = nullptr;
	}

	if (controllerVec.size() > 0)
	{
		for (auto it = controllerVec.begin(); it != controllerVec.end();)
		{
			BaseController* baseController = *it;
			if (baseController != nullptr)
			{
				delete (baseController);
				controllerVec.erase(it);
			}
			else
			{
				it++;
			}
		}
	}
#if HTTPSERVER_AUTO_CONTROLLER
	if (autoControllerVec.size() > 0)
	{
		for (auto it = autoControllerVec.begin(); it != autoControllerVec.end();)
		{
			AutoBashController* autoController = *it;
			if (autoController != nullptr)
			{
				delete (autoController);
				autoControllerVec.erase(it);
			}
			else
			{
				it++;
			}
		}
	}
#endif
}


inline void httplib::BaseController::replace_all(std::string& str, const std::string& old_value,
	const std::string& new_value)
{
	std::string::size_type pos(0);
	while ((pos = str.find(old_value)) != std::string::npos)
	{
		str.replace(pos, old_value.length(), new_value);
	}
}

inline void httplib::BaseController::initToServer(httplib::Server* _server)
{
	server = _server;
	// print("{} init to server", typeid(*this).name());
	this->bind();
}

inline void httplib::BaseController::initToServer(httplib::HttpServer* _server)
{

	server = _server->getHttplibServer();
	// print("{} init to server", typeid(*this).name());
	this->bind();
}

inline void httplib::BaseController::bind()
{
	throw std::runtime_error(std::string("HttpServer must override ") + __FUNCTION__);
}

inline httplib::BaseController::~BaseController()
{
	if (server != nullptr)
	{
		server = nullptr;
	}
	// print("destroy controller");
}


#if HTTPSERVER_AUTO_CONTROLLER

inline void httplib::HttpServer::addController(httplib::AutoBashController* controller)
{
	autoControllerVec.push_back(controller);
	controller->initToServer(this);
}


inline void httplib::AutoBashController::bind()
{
	if (httplib::AutoBashController::__autoBindFuncMap.size() == 0)
	{
		std::cout << "httplib::HttpServer[AutoBashController] warning: bind request path/func failed,  map size = 0, "
			"maybe you need use HttpServer.addController(your controller)\n";
	}
	std::unique_lock<std::mutex> ulock(__autoMapResetMtx);
	for (auto& it : httplib::AutoBashController::__autoBindFuncMap)
	{

		HttpServerRequestType requestType = it.second.first;
		std::function<void(const httplib::Request&, httplib::Response&)> func = it.second.second;
		switch (requestType)
		{
		case httplib::HttpServerRequestType::GET: {
			__httplib_server->Get(it.first, std::move(func));
			break;
		}
		case httplib::HttpServerRequestType::POST: {

			__httplib_server->Post(it.first, std::move(func));
			break;
		}

		case httplib::HttpServerRequestType::PUT: {
			__httplib_server->Put(it.first, std::move(func));
			break;
		}

		case httplib::HttpServerRequestType::PATCH: {
			__httplib_server->Patch(it.first, std::move(func));
			break;
		}
		case httplib::HttpServerRequestType::DEL: {
			__httplib_server->Delete(it.first, std::move(func));
			break;
		}
		case httplib::HttpServerRequestType::Options: {
			__httplib_server->Options(it.first, std::move(func));
			break;
		}
		}
	}
	__autoBindFuncMap.clear();
}

inline void httplib::AutoBashController::replace_all(std::string& str, const std::string& old_value,
	const std::string& new_value)
{
	std::string::size_type pos(0);
	while ((pos = str.find(old_value)) != std::string::npos)
	{
		str.replace(pos, old_value.length(), new_value);
	}
}

struct httplib::AutoBashController::PathStruct__ {
	PathStruct__(const HttpServerRequestType requestType, const std::string& path,
		const httplib::Server::Handler& requestFunc)

	{
		if (path.find_first_of('/') != 0)
		{
			std::string warn_info =
				std::string("httplib::HttpServer[AutoBashController] warning: error request path = ") + path +
				", maybe /\"" + path + "\" is right";
			std::cout << warn_info << std::endl;
		}
		httplib::AutoBashController::__autoBindFuncMap.emplace(path,
			std::make_pair(requestType, std::move(requestFunc)));
	}
};

struct httplib::AutoBashController::FilterStruct__ {
	FilterStruct__(const HttpServerFilterType filterType, AutoBashController* controllerObj,
		httplib::Server::Handler&& requestFunc)

	{
		if (filterType == httplib::HttpServerFilterType::REQUEST_FILTER)
		{
			controllerObj->__request_filter = std::move(requestFunc);
		}
		else if (filterType == httplib::HttpServerFilterType::RESPONCE_FILTER)
		{
			controllerObj->__response_filter = std::move(requestFunc);
		}
	}
};

inline void httplib::AutoBashController::initToServer(httplib::Server* _server)
{
	__httplib_server = _server;
	// print("{} init to server", typeid(*this).name());
	this->bind();
}

inline void httplib::AutoBashController::initToServer(httplib::HttpServer* _server)
{
	__httplib_server = _server->getHttplibServer();
	// print("{} init to server", typeid(*this).name());

	this->bind();
}

#endif


// 绑定宏
#if HTTPSERVER_AUTO_CONTROLLER
#define __HTTPLIB_CREATE_CONST_VAR_NAME(a, b) a##b
#define HTTPLIB_CREATE_CONST_VAR_NAME(a, b) __HTTPLIB_CREATE_CONST_VAR_NAME(a, b)
#define __RequestBind(requestType, requestFunc, url)                                                               \
        const PathStruct__ HTTPLIB_CREATE_CONST_VAR_NAME(p, __LINE__) = {                                              \
            httplib::requestType, url, [&](const httplib::Request& req, httplib::Response& resp) {                     \
                if (this->__request_filter != nullptr)                                                                 \
                    this->__request_filter(req, resp);                                                                 \
                this->requestFunc(req, resp);                                                                          \
                if (this->__response_filter != nullptr)                                                                \
                    this->__response_filter(req, resp);                                                                \
            }}
#define __FilterBind(filterType, filterFunc)                                                                       \
        const FilterStruct__ HTTPLIB_CREATE_CONST_VAR_NAME(p, __LINE__) = {                                            \
            httplib::filterType, this, [&](const httplib::Request& req, httplib::Response& resp) {                     \
                this->filterFunc(req, resp);                                                                           \
            }}

#define GetRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::GET, requestFunc, url)
#define PostRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::POST, requestFunc, url)
#define PutRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::PUT, requestFunc, url)
#define PatchRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::PATCH, requestFunc, url)
#define DeleteRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::DEL, requestFunc, url)
#define OptionRequestBind(requestFunc, url) __RequestBind(HttpServerRequestType::Options, requestFunc, url)
#define BeforeFilter(filterFunc) __FilterBind(HttpServerFilterType::REQUEST_FILTER, filterFunc)
#define AfterFilter(filterFunc) __FilterBind(HttpServerFilterType::RESPONCE_FILTER, filterFunc)
#endif
