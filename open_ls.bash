#!/bin/bash
# 停止 autodth 服务
sudo systemctl stop autodth.service

# 循环发送一号车CAN报文，间隔100ms
while true; do
    # cansend can0 80001344#80 00 00 00 00 00 00 00
    cansend can0 80001348#0300000000000000
    sleep 0.05
done
