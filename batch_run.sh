#!/bin/bash
count=10;
outfile="log.csv";
while getopts n:c:o: flag
do
    case "${flag}" in
        n) count=${OPTARG};;
        c) filename=${OPTARG};;
        o) outfile=${OPTARG};;
    esac
done

echo "count: $count";
echo "filename: $filename";

rm $outfile;

for i in $(seq $count); do
    output=$(argos3 -z -n -c $filename | tail -1);
    output2=`expr match "$output" '^\([0-9]*\)'`
    echo $output2;
    echo $output2 >> $outfile;
done