#!/bin/bash
"""
单个RViz配置测试脚本
快速测试指定的RViz配置文件
"""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 工作目录
WORKSPACE_DIR="/home/wb/ros_ws"

# 打印帮助信息
show_help() {
    echo "RViz配置测试脚本"
    echo ""
    echo "使用方法:"
    echo "  $0 <config_name>              # 测试指定配置"
    echo "  $0 --list                     # 列出所有可用配置"
    echo "  $0 --help                     # 显示帮助"
    echo ""
    echo "可用配置:"
    echo "  robot_view       # 基本机器人视图"
    echo "  slam_view        # SLAM建图视图"
    echo "  navigation_view  # 导航视图"
    echo "  full_navigation  # 完整导航视图"
    echo ""
    echo "示例:"
    echo "  $0 robot_view"
    echo "  $0 slam_view"
}

# 列出配置文件
list_configs() {
    echo -e "${BLUE}可用的RViz配置文件:${NC}"
    echo ""
    
    configs=(
        "robot_view:基本机器人视图"
        "slam_view:SLAM建图视图"
        "navigation_view:导航视图"
        "full_navigation:完整导航视图"
    )
    
    for config in "${configs[@]}"; do
        name=$(echo $config | cut -d: -f1)
        desc=$(echo $config | cut -d: -f2)
        file_path="$WORKSPACE_DIR/src/my_robot_description/rviz/${name}.rviz"
        
        if [ -f "$file_path" ]; then
            echo -e "  ${GREEN}✅ $name${NC} - $desc"
        else
            echo -e "  ${RED}❌ $name${NC} - $desc (文件不存在)"
        fi
    done
}

# 清理进程
cleanup() {
    echo -e "\n${YELLOW}清理后台进程...${NC}"
    pkill -f gzserver 2>/dev/null
    pkill -f gzclient 2>/dev/null
    pkill -f rviz2 2>/dev/null
    pkill -f robot_state_publisher 2>/dev/null
    sleep 2
}

# 测试RViz配置
test_config() {
    local config_name=$1
    local config_file="$WORKSPACE_DIR/src/my_robot_description/rviz/${config_name}.rviz"
    
    echo -e "${BLUE}🧪 测试RViz配置: $config_name${NC}"
    echo "配置文件: $config_file"
    
    # 检查配置文件是否存在
    if [ ! -f "$config_file" ]; then
        echo -e "${RED}❌ 配置文件不存在: $config_file${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ 配置文件存在${NC}"
    
    # 进入工作目录
    cd $WORKSPACE_DIR
    
    # 检查是否需要启动仿真
    if [[ "$config_name" != "empty" ]]; then
        echo -e "\n${YELLOW}🚀 启动仿真环境...${NC}"
        source install/setup.bash
        ros2 launch my_robot_simulation combined.launch.py gui:=false &
        SIM_PID=$!
        
        echo "等待仿真环境启动..."
        sleep 8
        
        # 检查话题是否可用
        echo "检查ROS话题..."
        source install/setup.bash
        TOPICS=$(ros2 topic list 2>/dev/null)
        
        if echo "$TOPICS" | grep -q "/scan"; then
            echo -e "${GREEN}✅ 激光雷达话题正常${NC}"
        else
            echo -e "${YELLOW}⚠️  激光雷达话题未找到${NC}"
        fi
        
        if echo "$TOPICS" | grep -q "/odom"; then
            echo -e "${GREEN}✅ 里程计话题正常${NC}"
        else
            echo -e "${YELLOW}⚠️  里程计话题未找到${NC}"
        fi
        
        if echo "$TOPICS" | grep -q "/camera/image_raw"; then
            echo -e "${GREEN}✅ 相机话题正常${NC}"
        else
            echo -e "${YELLOW}⚠️  相机话题未找到${NC}"
        fi
    fi
    
    # 启动RViz
    echo -e "\n${YELLOW}🎯 启动RViz...${NC}"
    echo "配置: $config_name"
    echo "按 Ctrl+C 退出测试"
    
    source install/setup.bash
    rviz2 -d "$config_file"
    
    # 用户退出RViz后清理
    cleanup
    echo -e "${GREEN}✅ 测试完成${NC}"
}

# 主函数
main() {
    # 检查参数
    if [ $# -eq 0 ]; then
        echo -e "${RED}错误: 请指定配置名称${NC}"
        show_help
        exit 1
    fi
    
    case $1 in
        --help|-h)
            show_help
            exit 0
            ;;
        --list|-l)
            list_configs
            exit 0
            ;;
        robot_view|slam_view|navigation_view|full_navigation)
            # 设置清理陷阱
            trap cleanup EXIT INT TERM
            test_config $1
            ;;
        *)
            echo -e "${RED}错误: 未知的配置名称 '$1'${NC}"
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"