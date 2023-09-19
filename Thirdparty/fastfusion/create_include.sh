#!/bin/sh
#This bash-script creates symbolic links to required headers 

SRCDIRS="$@"

printf "\033[34;1m-- -- creating symbolic links for headers \033[0m \n"

echo "SRCDIRS: $SRCDIRS"

if [ ! -d include ]; then
    mkdir -p include
fi

STARTING_DIR=`pwd`

for DIR in $SRCDIRS ; do
	#echo "DIR: $DIR"
	for SUBDIR in "$DIR"/* ; do
		echo "SUBDIR: $SUBDIR"
		# check if we are actually dealing with a directory 
		if [ -d $SUBDIR ]; then
			ln -sf $SUBDIR $STARTING_DIR/include
		fi
	done
done 

