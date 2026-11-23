# Ejecutar con: source ./entorno.sh (en cada terminal).
_g17_sprint3="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash || return 1
if [[ -f "$_g17_sprint3/install/setup.bash" ]]; then
    source "$_g17_sprint3/install/setup.bash" || return 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
unset _g17_sprint3
