DOCKER_USER="${USER}"
DEV_CONTAINER="smartsim_dev_${USER}"

xhost +local:root 1>/dev/null 2>&1

docker exec \
    -u "${DOCKER_USER}" \
    -e HISTFILE=/smartsim/.dev_bash_hist \
    -it "${DEV_CONTAINER}" \
    /bin/bash

xhost -local:root 1>/dev/null 2>&1