#!/bin/bash

# left, right, top, crop = L, R, T, C
# start, end, = S, E
# X,Y = X, Y

xSize=124
ySize=124

LXS=142;
LYS=343;

RXS=465;
RYS=344;

TXS=303;
TYS=183;

CXS=287;
CYS=328;
CXSize=155;
CYSize=155;

root=`pwd`;
templates="raw";
marker="whycon";

LXE=$((LXS+xSize));
LYE=$((LYS+ySize));
RXE=$((RXS+xSize));
RYE=$((LYS+ySize));
TXE=$((TXS+xSize));
TYE=$((TYS+ySize));
CXE=$((CXS+CXSize));
CYE=$((CYS+CYSize));

cd "$root";

for i in $(ls $templates |sed -e s/Slide//|sed -e s/.png//); do
    input_file=$templates/Slide$i.png;
    marker_file=$marker/$(printf "%08d.png" $i);
    x="x";
    p="+";
    ca=$CXSize$x$CYSize$p$CXS$p$CYS;

    convert $input_file -fill white -stroke white \
        -draw "rectangle $LXS,$LYS $LXE,$LYE" \
        -draw "rectangle $RXS,$RYS $RXE,$RYE" \
        -draw "rectangle $TXS,$TYS $TXE,$TYE" \
        -draw "rectangle $CXS,$CYS $CXE,$CYE" \
        cube.png;

    convert $input_file -crop $ca -trim +repage -resize 152x152\! pattern.png;

    convert cube.png pattern.png -geometry +$((CXS+2))+$((CYS+2)) -composite $i.png;

    convert $marker_file -resize $((xSize+20))$x$((ySize+20)) output.png;
    
    convert output.png -rotate 90 output.png;
    convert $i.png output.png -geometry +$((RXS-12))+$((RYS-12)) -composite $i.png;

    convert output.png -rotate 90 output.png;
    convert $i.png output.png -geometry +$((TXS-11))+$((TYS-11)) -composite $i.png;
    
    convert output.png -rotate 90 output.png;
    convert $i.png output.png -geometry +$((LXS-11))+$((LYS-11)) -composite $i.png;

    rm -f pattern.png cube.png output.png;

done
