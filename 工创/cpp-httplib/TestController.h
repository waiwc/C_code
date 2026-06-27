#pragma once

#include "HttpServer.h"
#include <mutex>
#include <string>
#include <unordered_set>

// 引入外部的共享集合和互斥锁
extern std::unordered_set<std::string>
received_data; // 使用集合来存储当前按下的按键
extern std::mutex data_mutex;

void removeSubstring(std::string &str, const std::string &toRemove) {
	// 查找并移除指定子串
	size_t pos = 0;
	while ((pos = str.find(toRemove, pos)) != std::string::npos) {
		str.erase(pos, toRemove.length());
	}
}

class TestController : public httplib::BaseController {
private:
	void bind() override {
		server->Post("/", BindController(&TestController::postApi, this));
	}

	// POST 请求处理，将数据存储到共享集合中
	void postApi(const httplib::Request &req, httplib::Response &resp) {
		std::lock_guard<std::mutex> lock(data_mutex); // 锁定互斥量以保护集合

		std::string temp = req.body;
		removeSubstring(temp, "keys="); // 移除 "keys=" 前缀

		// 检查请求中的状态
		if (temp.find("state=stear") != std::string::npos) {
			temp.erase(temp.find("&state=stear")); // 移除状态部分
			received_data.insert(temp);            // 将按键添加到集合中
		}
		else if (temp.find("state=endl") != std::string::npos) {
			temp.erase(temp.find("&state=endl")); // 移除状态部分
			received_data.erase(temp);            // 从集合中删除按键
		}

		//  // 打印当前集合中的按键
		//   for (const auto &key : received_data) {
		//     std::cout << key << " ";
		//  }
		//   std::cout << std::endl;

		   // 发送响应
		std::string jsonData = "{\"status\": \"received\", \"code\": 200}";
		resp.set_content(jsonData, "application/json");
	}
};
