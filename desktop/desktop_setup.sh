# Define container name
CONTAINER_NAME="desktop-controller"

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

# Initialize ROS_DOMAIN_ID to 0 by default
ROS_DOMAIN_ID=0
HUSARNET_HOSTNAME=duke-desktop

# Check if ROS_DOMAIN_ID and HUSARNET_JOIN_CODE flags are provided
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 -j <HUSARNET_JOIN_CODE> [-d <ROS_DOMAIN_ID>] -o <OPERATOR_ID>"
    exit 1
fi

# Parse command-line arguments
while getopts ":j:d:o:" opt; do
    case ${opt} in
        j )
            HUSARNET_JOIN_CODE=$OPTARG
            ;;
        d )
            ROS_DOMAIN_ID=$OPTARG
            ;;
        o )
            HUSARNET_HOSTNAME=$OPTARG
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

export HUSARNET_JOIN_CODE
export HUSARNET_HOSTNAME
export ROS_DOMAIN_ID
export CONTAINER_NAME


echo "COMPOSE_HUSARNET_JOIN_CODE=$HUSARNET_JOIN_CODE" > .env
{
echo "COMPOSE_HUSARNET_HOSTNAME=$HUSARNET_HOSTNAME"
echo "COMPOSE_ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "COMPOSE_CONTAINER_NAME=$CONTAINER_NAME"
} >> .env


# Check if the containers managed by the specified docker compose file exist
if sudo docker ps -a --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    confirm "Containers defined in the $CONTAINER_NAME compose.yaml file will be destroyed. Do you want to proceed?" && \
    sudo docker compose -f ./compose.yaml down
fi

# Build and start the container
sudo docker compose -f ./compose.yaml up -d --build
containerinstructions
