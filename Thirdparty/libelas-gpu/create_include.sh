#!/bin/sh
#This bash-script creates symbolic links to required headers 

SRCDIRS="$@"

printf "\033[34;1m-- -- creating symbolic links for headers \033[0m \n"

echo "SRCDIRS: $SRCDIRS"

if [ ! -d include ]; then
    mkdir include
fi

STARTING_DIR=`pwd`

for DIR in $SRCDIRS ; do
	echo "DIR: $DIR"
	for file in "$DIR"/* ; do
		#echo "file src: $file"
		file_dst=$STARTING_DIR/include/$(basename $file)
		#echo "file dst: $file_dst"
		if [ ! -f $file_dst ]; then
			ln -sf $file $STARTING_DIR/include/
		fi
	done
done 
