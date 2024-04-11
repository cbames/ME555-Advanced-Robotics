#!/usr/bin/env bash
# Define container name
CONTAINER_NAME="operator"
COMPOSE_SERVER_NAME="duke-desktop"

# Function to prompt for confirmation
confirm() {
    read -r -p "$1 [y/N] " response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            true
            ;;
        *)
            false
            ;;
    esac
}

# Instructions for how to interact with container
containerinstructions() {
    echo "To open a bash shell in the container, run:"
    echo "docker exec -it $CONTAINER_NAME bash"
}

# Check if ROS_DOMAIN_ID and HUSARNET_JOIN_CODE flags are provided
#if [[ $# -lt 1 ]]; then
#    echo "Usage: $0  [-d <ROS_DOMAIN_ID>] [-t <SERVER_NAME>] [-n OPERATOR_NAME]"
#    exit 1
#fi

# Parse command-line arguments
while getopts ":j:d:t:n:s:" opt; do
    case ${opt} in
        d )
            COMPOSE_ROS_DOMAIN_ID=$OPTARG
            ;;
        n )
            OPERATOR_NAME=$OPTARG
            ;;
        \? )
            echo "Invalid option: $OPTARG" 1>&2
            exit 1
            ;;
        : )
            echo "Invalid option: $OPTARG requires an argument" 1>&2
            exit 1
            ;;
    esac
done
shift $((OPTIND -1))

# Check if there are any unexpected arguments or if the -n flag is used incorrectly
if [ "$#" -gt 0 ] || { [ -z "$OPERATOR_NAME" ] && [ "$OPTIND" -eq 2 ]; }; then
    usage
fi

# If OPERATOR_NAME is not provided, generate a random one
if [ -z "$OPERATOR_NAME" ]; then
    OPERATOR_NAME=$(tr -dc 'a-zA-Z0-9' < /dev/urandom | fold -w 16 | head -n 1)
fi

echo "Operator Name: $OPERATOR_NAME"

echo "COMPOSE_HUSARNET_JOIN_CODE=$COMPOSE_HUSARNET_JOIN_CODE" > .env
{
echo "COMPOSE_HUSARNET_HOSTNAME=operator-$OPERATOR_NAME"
echo "COMPOSE_ROS_DOMAIN_ID=$COMPOSE_ROS_DOMAIN_ID"
echo "COMPOSE_SERVER_NAME=$COMPOSE_SERVER_NAME"
} >> .env



# Check if the containers managed by the specified docker compose file exist
if sudo docker ps -a --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    confirm "Containers defined in the $CONTAINER_NAME compose.yaml file will be destroyed. Do you want to proceed?" && \
    sudo -E docker compose -f ./compose.yaml down
    xhost -local:
fi

# Build and start the container
sudo -E docker compose -f ./compose.yaml up -d --build 

containerinstructions
