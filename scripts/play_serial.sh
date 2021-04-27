#!/bin/bash

# Function to correctly catch the Ctrl-C
function finish() {
  echo -e "\nBye bye!"
  exit
}

trap finish SIGINT

# Taking the input file from the first command line argument
input=$1
count=0

if test -f "$input"; then
  # Displaying the input file name
  echo "Logfile: $input"
else 
  echo "'$input' file does not exist."
  echo "Provide file as 1st command line argument."
  exit
fi

while [ 1 ]
do
  ((count++))
  echo "Started playing logfile... $count"
  # Using pv to limit the throughput of the cat command to 80 bytes/s
  cat $input | pv -l -L 80 -q > /tmp/serial2

done

