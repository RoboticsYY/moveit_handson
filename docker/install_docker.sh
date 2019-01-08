#!/bin/bash
set -e

sudo apt-get remove docker docker-engine docker.io containerd runc

# ubuntu 16.04 and newer
# 1.Update the apt package index:
sudo apt-get update

# 2.Install packages to allow apt to use a repository over HTTPS:
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
# 3.Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
# 4.Use the following command to set up the stable repository. x86_64 / amd64
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
# 5.Update the apt package index.
sudo apt-get update
# 6.Install docker-ce
sudo apt-get install -y docker-ce
apt-cache madison docker-ce
# 7.add user group
sudo groupadd docker
sudo usermod -aG docker $USER
