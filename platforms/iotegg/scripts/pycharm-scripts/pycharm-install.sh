#!/bin/bash
rm pycharm-community-5.0.4.tar.gz 
sudo mv pycharm-community-5.0.4 /opt/pycharm-community-5.0.4
echo " " >> ~/.bashrc
echo "# PyCharm Installation" >> ~/.bashrc
echo "export PATH="\$PATH:/opt/pycharm-community-5.0.4/bin/"" >> ~/.bashrc
