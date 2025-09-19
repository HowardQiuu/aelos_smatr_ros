import time
import json
import os

def main():
    time.sleep(30)

    with open("/home/lemon/NetworkManager/wifi_msg.json", "r") as f:

        data = json.loads(f.read())

        connect_wifi_cmd = "sudo nmcli dev wifi connect '{name}' password {password}".format(name=data["name"], password=data["password"])

        os.system(connect_wifi_cmd)


if __name__ == '__main__':
    main()
