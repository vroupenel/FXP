while read state
do
	echo $state
	if [[ $state == *"SLEEPING"* ]]; then
		echo zzz
	fi
	if [[ $state == *"WAKING_UP-ENTRY"* ]]; then
		echo Look around
	fi
done
