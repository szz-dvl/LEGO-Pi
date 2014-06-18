#!/bin/bash

# This script is the LEGO-Pi library installer. overclock, HDMI and LIBS export Missing [16-5-2014]


mydir=`pwd`
distro=`cat /etc/*-release | grep NAME | grep -v PRETTY | awk -F= {'print $2'} | tr -d \"`
mylib=/usr/lib/liblego.so
deps=( gsl-bin libgsl0-dev gcc patch ctags git-core make )
depsu=( gsl-bin libgsl0-dev patch ctags git-core )
wpi_sec="$mydir"/patch/files/wpil.tar.gz
wpi_git=git://git.drogon.net/wiringPi
wpi_patch="$mydir"/patch/wpi.patch
lego_pi="$mydir"/../
wpi="$mydir"/wpi/wiringPi
wpi_ins="$mydir"/wpi
wpi_dev="$mydir"/wpi/devLib
wpi_gpio="$mydir"/wpi/gpio
patch_files=( wiringPi.c wiringPi.h softPwm.c softPwm.h )
patch_dir="$mydir"/patch/files
tmp_d="$mydir"/patch/tmp

ndone="Nothing done, exiting"

make_patch()
{

    mkdir "$tmp_d"
    let "i=0"
    rm "$wpi_patch"

    for file in "${patch_files[@]}"; do
	diff -u "$wpi"/"$file" "$patch_dir"/"$file" > "$tmp_d"/p"$i".patch
	let "i=i+1"
    done
    
    cat "$tmp_d"/p0.patch "$tmp_d"/p1.patch "$tmp_d"/p2.patch "$tmp_d"/p3.patch > "$wpi_patch"
    rm -r "$tmp_d"
}

bye()
{
    echo ""$0": "$1"" >&2
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
dvideo=true     #disable video outputs PAL/HDMI by default
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
	sudo rm -r "$wpi_ins"
    fi

    if ! [ -d "$wpi_ins" ]; then
	git clone "$wpi_git" "wpi"
	
	if ! [ -d "$wpi_ins" ]; then
	    mkdir "$wpi_ins"
	    bold "Unable to fetch wiringPi, using a local copy, it might be outdated ...\n"
	    tar -xvzf "$wpi_sec" -C "$wpi_ins" >/dev/null
	fi
	
    else
	cd "$wpi_ins"
	git pull origin
    fi

    make_patch
    cd "$wpi"
    patch -N -s -p7 < "$wpi_patch"

    cd "$wpi_ins"
    ./build
    
    cd "$lego_pi"
    if [ -e "$mylib" ]; then
	sudo make uninstall
    fi
    
    sudo make i2c && make && sudo make install && make test && make clean
    cd "$mydir"

    bold "\nLEGO_Pi successfully installed, check for errors if this is not the case." ; echo "You will find the documentation in de the \"docs\" direcory"

else #uninstall

    bold "\nUninstalling LEGO-Pi ..."
    
    if [[ "$rmv_deps" = "true" ]]; then

	bold "\nUninstalling dependences ..."

	if [ -d "$wpi_ins" ]; then
	    cd "$wpi"
            sudo make uninstall
	    cd "$wpi_dev"
            sudo make uninstall
	    cd "$wpi_gpio"
	    sudo make uninstall

	    cd "$mydir"
	    sudo rm -r "$wpi_ins"
	fi
	
	if [[ "$pkg_man" == "apt-get" ]]; then
	    sudo "$pkg_man" "--yes" "remove" ${depsu[*]}
	else
	    sudo "$pkg_man" ${depsu[*]}
	fi
	
    elif [ -e "$mylib" ]; then
	
	bold "\nUnpatching wiringPi ..."
	cd "$wpi"
	patch -R -N -p7 < "$wpi_patch"
	cd "$wpi_ins"
	./build
	
    fi
    
    cd "$lego_pi"
    sudo make uninstall
    
    cd "$mydir"
    
    if [ -e "$mylib" ]; then
	rm "$wpi_patch"
    fi

    bold "\nLEGO_Pi successfully uninstalled, check for errors if this is not the case."
    #Quitar seds, por lo menos de bashrc...
    
fi
exit 0
