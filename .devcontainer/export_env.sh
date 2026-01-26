#!/bin/bash
# 导出用户信息作为环境变量，供 Docker Compose 使用

export USER_NAME=$(whoami)
export USER_ID=$(id -u)
export GROUP_NAME=$(id -gn)
export GROUP_ID=$(id -g)

echo "已设置以下环境变量："
echo "USER_NAME=$USER_NAME"
echo "USER_ID=$USER_ID"
echo "GROUP_NAME=$GROUP_NAME"
echo "GROUP_ID=$GROUP_ID"
echo ""
echo "现在可以运行: docker compose -f docker-compose.dev.yaml up"

