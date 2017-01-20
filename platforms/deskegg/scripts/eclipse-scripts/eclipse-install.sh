#!/bin/bash
rm eclipse-cpp-mars-2-linux-gtk-x86_64.tar.gz
sudo mv eclipse /opt/eclipse
echo " " >> ~/.bashrc
echo "# Eclipse Installation" >> ~/.bashrc
echo "export PATH="\$PATH:/opt/eclipse"" >> ~/.bashrc
