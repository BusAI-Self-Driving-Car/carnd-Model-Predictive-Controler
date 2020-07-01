#! /bin/bash
sudo apt-get update
sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
sudo apt-get install gfortran
sudo apt-get install unzip

wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip && unzip Ipopt-3.12.8.zip && rm Ipopt-3.12.8.zip
sudo ./install_ipopt.sh Ipopt-3.12.8
sudo rm -rf Ipopt-3.12.8

sudo apt-get install cppad
