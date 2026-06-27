#pragma once
#include "EnvironmentVariable.hpp"
#include "BuildVersion.hpp"
#include "ProductVersion.hpp"
#include <memory>

namespace cybertron {}

#define CybertronCoreAPI

#ifndef CYBERTRON_BEGIN
#define CYBERTRON_BEGIN namespace cybertron {
#endif

#ifndef CYBERTRON_END
#define CYBERTRON_END }
#endif

CYBERTRON_BEGIN

struct vec2;
struct vec3;
struct vec4;
struct bvec2;
struct bvec3;
struct bvec4;
struct ivec2;
struct ivec3;
struct ivec4;
struct dvec2;
struct dvec3;
struct dvec4;
struct mat3;
struct dmat3;
struct mat4;
struct dmat4;
struct quat;
struct dquat;
struct box3;
struct dbox3;

class Message;
class SocketBase;
class SocketEventHandler;

class SocketTcpClient;
class SocketTcpClientSync;
class SocketTcpServer;

typedef std::shared_ptr<SocketBase> SocketBasePtr;
typedef std::shared_ptr<SocketEventHandler> SocketEventHandlerPtr;
typedef std::shared_ptr<SocketTcpClient> SocketTcpClientPtr;
typedef std::shared_ptr<SocketTcpServer> SocketTcpServerPtr;

class NodeHotAreaManager;
class NodeInternalServer;
class NodeHotAreaChangeRecorder;
class NodeAppBase;

class PropertyOwner;
class PropertyVisitor;

CYBERTRON_END

namespace Coordinator {
	class FromAnyRequestTaskStart;
}

namespace google {
	namespace protobuf {
		class MessageLite;
	}
}

// Platform defines
#if defined(CYBERTRON_WIN)
#undef CYBERTRON_WIN
#endif

#if defined(CYBERTRON_LINUX)
#undef CYBERTRON_LINUX
#endif

#if defined(WIN32) || defined(_WIN32)
  #define CYBERTRON_WIN 1
//#include <asio/asio.hpp>
//#include <windows.h>
#elif defined(__linux__) || defined(__linux)
  #define CYBERTRON_LINUX 1
#elif defined(__APPLE__)
  #include "TargetConditionals.h"
  #ifdef TARGET_OS_IPHONE
    #define CYBERTRON_IOS 1
  #elif TARGET_IPHONE_SIMULATOR
    #define CYBERTRON_IOS_SIM 1
  #elif TARGET_OS_MAC
    #define CYBERTRON_MAC 1
  #else
    #error "Unsupported Apple OS!"
  #endif
  
#else
  #error "Unknow OS not supported!"
#endif


// Opendrive definations


#define ODV0001 "51vr-od-v0001"