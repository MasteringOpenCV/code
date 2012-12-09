#!/bin/bash
echo "#ITS \t 5 \t 10 \t 15 \t 20" > data.txt
folder=$(pwd)

for i in 10 20 30 40 50 60 70 80 90 100 120 150 200 500
do
	s5=0;
	s10=0;
	s15=0;
	s20=0;
	for j  in {1..100}
	do
		echo $i $j
		a=$($folder/build/evalOCR $i TrainingDataF5)
		s5=$(echo "scale=4; $s5+$a" | bc -q 2>/dev/null)

		a=$($folder/build/evalOCR $i TrainingDataF10)
		s10=$(echo "scale=4; $s10+$a" | bc -q 2>/dev/null)

		a=$($folder/build/evalOCR $i TrainingDataF15)
		s15=$(echo "scale=4; $s15+$a" | bc -q 2>/dev/null)

		a=$($folder/build/evalOCR $i TrainingDataF20)
		s20=$(echo "scale=4; $s20+$a" | bc -q 2>/dev/null)
	done	
	
	echo "$i \t $s5 \t $s10 \t $s15 \t $s20"
	echo "$i \t $s5 \t $s10 \t $s15 \t $s20" >> data.txt
done

