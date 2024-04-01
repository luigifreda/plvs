#!/usr/bin/env bash

# a collection of bash utils 


function print_blue(){
    printf "\033[34;1m"
    printf "$@ \n"
    printf "\033[0m"
}

function print_red(){
    printf "\033[31;1m"
    printf "$@ \n"
    printf "\033[0m"
}

function make_dir(){
    if [ ! -d $1 ]; then
        mkdir $1
    fi
}

function make_buid_dir(){
    make_dir build
}

function check_package(){
    package_name=$1
    PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $package_name |grep "install ok installed")
    #echo "checking for $package_name: $PKG_OK"
    if [ "" == "$PKG_OK" ]; then
      #echo "$package_name is not installed"
      echo 1
    else
      echo 0
    fi
}
function install_package(){
    do_install=$(check_package $1)
    if [ $do_install -eq 1 ] ; then
        sudo apt-get install -y $1
    fi 
}

function install_packages(){
    for var in "$@"
    do
        install_package "$var"
    done
}

function check_pip_package(){
    package_name=$1
    PKG_OK=$(pip3 list |grep $package_name)
    #print_blue "checking for package $package_name: $PKG_OK"
    if [ "" == "$PKG_OK" ]; then
      #print_blue "$package_name is not installed"
      echo 1
    else
      echo 0
    fi
}
function install_pip_package(){
    do_install=$(check_pip_package $1)
    if [ $do_install -eq 1 ] ; then
        pip3 install --user $1
    fi 
}
function install_pip_packages(){
    for var in "$@"
    do
        install_pip_package "$var"
    done
}


function extract_version(){
    #version=$(echo $1 | sed 's/[^0-9]*//g')
    #version=$(echo $1 | sed 's/[[:alpha:]|(|[:space:]]//g')
    version=$(echo $1 | sed 's/[[:alpha:]|(|[:space:]]//g' | sed s/://g)
    echo $version
}


function get_cuda_version(){
    if [ -d /usr/local/cuda ]; then 
        if [ -f /usr/local/cuda/version.txt ]; then 
            CUDA_STRING=$(cat /usr/local/cuda/version.txt)
            CUDA_VERSION=$(extract_version "$CUDA_STRING")
            echo $CUDA_VERSION
        else
            # Extract the CUDA version from the nvidia-smi output
            CUDA_VERSION=$(/usr/local/cuda/bin/nvcc --version | grep release | awk '{print $5}' | sed 's/,//')
            echo $CUDA_VERSION
        fi 
    else
        echo 0
    fi
}

function get_current_nvidia_driver_version(){
    NVIDIA_DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | cut -d. -f1)
    echo $NVIDIA_DRIVER_VERSION
}


# Function to compare two version strings
# Returns:
#   0 if versions are equal
#   1 if version1 > version2
#   2 if version1 < version2
function version_compare() {
    local v1=$1
    local v2=$2
    if [[ $v1 == $v2 ]]; then
        echo 0
        exit
    fi
    local IFS=.
    local a1=($v1)
    local a2=($v2)
    local len=$(( ${#a1[@]} > ${#a2[@]} ? ${#a1[@]} : ${#a2[@]} ))
    for ((i=0; i<len; i++)); do
        local part1=${a1[i]}
        local part2=${a2[i]}
        # If a version part is missing, consider it as 0
        [[ -z $part1 ]] && part1=0
        [[ -z $part2 ]] && part2=0
        if ((part1 > part2)); then
            echo 1
            exit
        elif ((part1 < part2)); then
            echo 2
            exit
        fi
    done
    echo 0
}
