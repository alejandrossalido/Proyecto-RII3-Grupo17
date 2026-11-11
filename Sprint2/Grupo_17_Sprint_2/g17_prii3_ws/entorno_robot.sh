# Usar con: source ./entorno_robot.sh  (en cada terminal SSH del robot).
# ROS_DISTRO puede fijarse antes si hay mas de una distribucion instalada.
_g17_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_g17_distro="${ROS_DISTRO:-}"
if [[ -z "$_g17_distro" ]]; then
    if [[ -f /opt/ros/foxy/setup.bash ]]; then
        _g17_distro=foxy
    elif [[ -f /opt/ros/humble/setup.bash ]]; then
        _g17_distro=humble
    else
        echo 'No se encuentra ROS Foxy/Humble en este equipo.' >&2
        return 1
    fi
fi
source "/opt/ros/$_g17_distro/setup.bash" || return 1
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
    source "$HOME/ros2_ws/install/setup.bash" || return 1
fi
if [[ -f "$_g17_ws/install/setup.bash" ]]; then
    source "$_g17_ws/install/setup.bash" || return 1
fi
export ROS_DOMAIN_ID=17
export ROS_LOCALHOST_ONLY=1
unset _g17_ws _g17_distro
