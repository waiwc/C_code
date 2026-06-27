import requests
import keyboard

# 定义目标URL，本地端口为5000（可以修改为实际需要的端口）
url = "http://127.0.0.1:9899"

# 用于跟踪每个按键的状态，防止重复发送相同状态的请求
key_states = {}


# 定义按下和释放事件的处理函数
def send_post_request(event):
    action = "stear" if event.event_type == "down" else "endl"

    # 检查按键是否已经处于所需状态
    if key_states.get(event.name) != action:
        # 更新按键状态
        key_states[event.name] = action
        # 设置 POST 数据格式
        post_data = {"keys": event.name, "state": action}

        # 发送 POST 请求
        response = requests.post(url, data=post_data)
        print(
            f"Sent POST request with data: {post_data}, Response status: {response.status_code}"
        )


# 使用 keyboard.hook 捕获按键的按下和释放事件
keyboard.hook(send_post_request)

# 保持脚本运行
keyboard.wait()
