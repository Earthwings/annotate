#!/bin/bash
version="$(glxinfo | grep "OpenGL version string" | rev | cut -d" " -f1 | rev)"
wget "http://us.download.nvidia.com/XFree86/Linux-x86_64/${version}/NVIDIA-Linux-x86_64-${version}.run" -O NVIDIA-DRIVER.run
