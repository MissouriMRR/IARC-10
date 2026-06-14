#!/usr/bin/bash

compose=podman-compose
oci_cmd=podman

function find_container_name() {
    # find container name that matches passed argument
    test=$(eval "$oci_cmd ps --format '{{.Names}}' | grep $1")
    echo $test  # return matched container name
}

function usage() {
    echo This script is intended as a simple wrapper for running the env and sim podman containers.
    echo
    echo Usage:
    echo "  cmd [command]"
    echo
    echo Available Commands:
    echo "      [all]          start both the sim and env containers"
    echo "      shutdown       shutdown both containers if any are running"
    echo "      sim            start sim only"
    echo "      env            start env only"
    echo "  h,  help           print this"
    echo "      restart [all]  restart both the sim and env containers"
    echo "      restart sim    restart the sim container"
    echo "      restart env    restart the env container"
    echo "      attach [env]   attach the env container (it must be running)"
    echo "      attach sim     attach the sim container (it must be running)"
    echo
}

# very simple command implementation
case $1 in
    ""|all)
        eval "$compose up -d"
    ;;
    shutdown)
        eval "$compose down"
    ;;
    sim)
        eval "$compose run --rm sim"
    ;;
    env)
        eval "$compose run --rm env"
    ;;
    h|help)
        usage
    ;;
    restart)
        case $2 in
            ""|all)
                eval "$compose restart"
            ;;
            sim)
                eval "$compose restart sim"
            ;;
            env)
                eval "$compose restart env"
            ;;
            *)
                echo "Could not restart '$2'"
                usage
            ;;
        esac
    ;;
    attach)
        case $2 in
            ""|env)
                container=$(find_container_name env)
                eval "$oci_cmd attach $container"
            ;;
            sim)
                container=$(find_container_name sim)
                eval "$oci_cmd attach $container"
            ;;
            *)
                echo "Could not attach to '$2'"
                usage
            ;;
        esac
    ;;
    *)
        echo "Unrecognized command '$1'"
        usage
    ;;
esac
