# Description:

Files in this folder is used to make docker image of intel/kinetic:moveit_handson base environment.

Dockerfile -- the base file which will be automatically loaded when run docker build command.

setup_docker_display.sh -- set the container to display a pop-up x window.

install_docker.sh -- install docker-ce on ubuntu16.04 and newer.

# Creating a docker image
## Install docker
First, clone the repo to your local system.
```
git clone https://github.com/RoboticsYY/moveit_handson.git
./docker/install_docker.sh 
```
## Build command:
```
cd moveit_handson/docker/
docker build -t intel/kinetic:moveit_handson_demo .
```
If proxies are needed:
```
docker build -t intel/kinetic:moveit_handson_demo --build-arg http_proxy=http://proxy.my.com:prot_number --build-arg https_proxy=http://proxy.my.com:prot_number .
```
## Load docker image
If you got our docker image, please follow description to do at below.
```
sudo apt update && sudo apt install -y tar
tar -zxvf moveit_handson_demo.tar.tgz
docker load < moveit_handson_demo.tar
```
## OPTION:Please refer below command to verify image creating success
```
docker images

REPOSITORY          TAG                   IMAGE ID            CREATED             SIZE
intel/kinetic       moveit_handson_demo   6643cc4db2e5        5 hours ago         3.47GB
```

# Run docker image
After the project runs, there will be a pop-up x window, you need to set the operating environment first.
```
./docker/setup_docker_display.sh

docker run -t -i --rm -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY --name moveit_handson intel/kinetic:moveit_handson_demo bash
```
Run this command if you want to open multiple container terminals
```
docker exec -t -i moveit_handson bash
```
Note: If you haven't already installed or want more information on how to use docker, please see the article here for more information:
https://docs.docker.com/install/
