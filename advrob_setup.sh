
helpmessage()
{
    echo "This script will build a development and control environment for the Advanced Robotics project"
    echo "usage:"
    echo "    --help, -h: print this message"
    echo "    --advrob-user, -u: your first name and last name"
    echo "    --joincode, -j: needed for remote teleoperation" 
}


instructions()
{
    echo "To start your dev environment container, run:"
    echo "docker exec -it operator-operator-1 bash"
}


# default values
ADVROB_USER=$USER


flags()
{
    local advrob_user_defined=false
    local joincode_defined=false

    while test $# -gt 0
    do
        case "$1" in
        -h|--help)
            helpmessage
            exit 0
            ;;
        -u|--advrob-user)
            if [ -z "$2" ]; then
                echo "ERROR: No username provided"
                helpmessage
                exit 1
            fi
            export ADVROB_USER=$2
            advrob_user_defined=true
            shift
            ;;
        -j|--joincode)
            if [ -z "$2" ]; then
                echo "ERROR: No JOINCODE provided"
                helpmessage
                exit 1
            fi
            export JOINCODE=$2
            joincode_defined=true
            shift
            ;;
        esac
        shift
    done

    # Check if both mandatory flags are provided
    if [ "$advrob_user_defined" != true ] || [ "$joincode_defined" != true ]; then
        echo "ERROR: Both -u|--advrob-user and -j|--joincode are mandatory flags"
        helpmessage
        exit 1
    fi
}
flags "$@"

docker compose -f ./operator/docker-compose.yaml up -d
instructions
