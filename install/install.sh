#!/bin/bash

# This script is the LEGO-Pi library installer.

bye()
{
    echo ""$0": "$1""
    exit 1
}

update()
{
    if [[ "$1" == "apt-get" ]]; then
        sudo "$1" "--yes" "update" 
	sudo "$1" "--yes" "upgrade"
    else
	sudo "$1""u"
    fi
}

bold ()
{

    echo -e "\e[1m"$1"\e[0m"


}


mydir=`pwd`
distro=`cat /etc/*-release | grep NAME | grep -v PRETTY | awk -F= {'print $2'} | tr -d \"`
mylib=/usr/lib/liblego.so
deps=(gsl-bin libgsl0-dev gcc patch ctags git-core)
wpi_git=git://git.drogon.net/wiringPi
wpi_patch="$mydir"/patch/wpi.patch
lego_pi="$mydir"/../
wpi="$mydir"/wpi/wiringPi
wpi_ins="$mydir"/wpi/
wpi_dev="$wpi_ins"/devLib
wpi_gpio="$wpi_ins"/gpio

ndone="Nothing done, exiting"

if [ "$distro" == "Raspbian GNU/Linux" ]; then
    pkg_man=apt-get
elif [ "$distro" == "Arch-algo" ]; then
    pkg_man=pacman -Sy
fi


max_clk=1000
wrn_clk=950
min_clk=700

# DEFAULTS:
uninstall=false #install by default
overclock=800   #little bit of overclocking by default 
dvideo=true    #disable video outputs PAL/HDMI by default
rmv_deps=false  #do not remove dependencies by when uninstalling default 

while [[ $# -gt 0 ]]; do
    opt="$1"
    shift;
    case "$opt" in
        "-u"|"--uninstall"  ) uninstall=true;;
        "-o"|"--oc"         ) overclock="$1"; shift;;
        "-r"|"--remove"     ) rmv_deps=true;;
        "-v"|"--video"      ) dvideo=false;;
        *                   ) echo "ERROR: Invalid option: \""$opt"\"" >&2
                              bye "$NDONE";;
    esac
done

if ! [[ $overclock =~ ^[0-9]+$ ]] || ([[ $overclock -lt $min_clk || $overclock -gt $max_clk ]]); then
    
    echo "ERROR: Overclock needs to be an integer rangin from "$min_clk" to "$max_clk"" >&2
    bye "$NDONE"
    
elif [[ $overclock -ge $wrn_clk ]]; then
    
    echo "WARNING: "$overclok" Hz overclocking may damage your Pi, continuing."
    
fi  

if ! [[ "$uninstall" == "true" ]]; then

    ##FALTAN seds

    bold "\nUpdating package database ..."
    update "$pkg_man"
    bold "\nInstalling depencdences ..."
    if [[ "$pkg_man" == "apt-get" ]]; then
	sudo "$pkg_man" "--yes" "install" ${deps[*]}
    else
	sudo "$pkg_man" "deps"
    fi


    if [ -d "$wpi_ins" ]; then
	rm -r "$wpi_ins"
    fi

    git clone "$wpi_git" "wpi"
    cd "$wpi"
    patch -N -R -p2 < "$wpi_patch"
    cd "$wpi_ins"
    ./build
    
    cd "$lego_pi"
    if [ -e "$mylib" ]; then
	sudo make uninstall
    fi
    
    sudo make i2c && make && sudo make install && make test && make clean
    cd "$mydir"
     
else #uninstall
    bold "\nUninstalling LEGO-Pi ..."
    
    if [[ "$rmv_deps" = "true" ]]; then

	bold "\nUninstalling dependences ..."

	if [[ "$pkg_man" == "apt-get" ]]; then
	    sudo "$pkg_man" "--yes" "remove" ${deps[*]}
	else
	    sudo "$pkg_man" ${deps[*]}
	fi

	cd "$wpi"
        sudo make uninstall
	cd "$wpi_dev"
        sudo make uninstall
	cd "$wpi_gpio"
	sudo make uninstall
	
	cd "$mydir"
	rm -r "$wpi_ins"

    elif [ -e "$mylib" ]; then
	bold "\nUnpatching wiringPi ..."
	cd "$wpi"
	patch -N -s -p2 < "$wpi_patch"
	cd "$wpi_ins"
	./build
	
    fi
    
    cd "$lego_pi"
    sudo make uninstall
    
    cd "$mydir"
    
    #Quitar seds, por lo menos de bashrc...
    
fi

exit 0
