#!/bin/zsh

# Inicializa o ambiente ROS 2
source /opt/ros/humble/setup.zsh

# Compila o pacote desejado
colcon build --packages-select navigation

# Inicializa o ambiente ROS 2
source ./install/setup.zsh

# Default map path
DEFAULT_MAP_PATH="./assets/maps/map.yaml"

launch_process(){
# Check if a map path is provided
    if [ -z "$1" ]; then
        echo "No map path provided. Using the default map path: $DEFAULT_MAP_PATH"
        MAP_PATH=$DEFAULT_MAP_PATH
    else
        MAP_PATH=$1
    fi

        ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAP_PATH" &
        ros2 launch ./src/navigation/launch/navigation_launch.py &

}

handle_ctrl_c() {
    echo "CTRL+C pressionado."

    # Encerra o processo ROS se ainda estiver em execução
    if kill -0 $ros_pid > /dev/null 2>&1; then
        kill_ros_process
    fi

    exit 0
}

# Define o manipulador para SIGINT (CTRL+C)
trap handle_ctrl_c SIGINT

# Lança o processo ROS
launch_process

# Aguarda o processo ROS terminar
wait $ros_pid
