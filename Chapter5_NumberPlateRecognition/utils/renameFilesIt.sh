#!/bin/bash
FILES=./*.jpg
FILES1=./*.JPG
i=0
for f in $FILES
do
    echo "Processing $f file..."
    # take action on each file. $f store current file name
    mv $f $i.jpg
    i=`expr $i + 1`
done

for f in $FILES1
do
    echo "Processing $f file..."
    # take action on each file. $f store current file name
    mv $f $i.jpg
    i=`expr $i + 1`
done
