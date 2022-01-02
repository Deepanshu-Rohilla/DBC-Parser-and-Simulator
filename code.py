import sys
import csv

logs = []
signalValues = {}
signalValues["vehiclespeed"] = 1
signalValues["indicator"] = 2
signalValues["locking"] = 3
signalValues["enginerpm"] = 4


def getName(signal):
	signal = signal.lower()
	print("Signal is ")
	print(signal)
	if "vehiclespeed" in signal:
		return 1
	elif "indicator" in signal:
		return 2
	elif "locking" in signal:
		return 3
	elif "enginerpm" in signal:
		return 4
	else:
		return -1


def getValue(first, second):
	canframe = [
		'FF', 
		'FF', 
		'FF', 
		'68',
		'13',
		'FF', 
		'FF', 
		'FF'
	] 
	hexVal = canframe[(first+second-1)//8] + canframe[first//8]
	return int(hexVal,16)


def extractValue(magnitude,scaleAndOffset):
	first = 0
	second = 0
	scale = 0
	offset = 0
	i = 0
	while(magnitude[i]!='|'):
		first = first*10 + int(magnitude[i])
		i = i +1
	i = i + 1
	while(magnitude[i]!='@'):
		second = second*10 + int(magnitude[i])
		i = i +1
	print(first)
	print(second)
	i = 1
	j = i
	while(scaleAndOffset[j]!=','):
		j = j +1
	scale = float(scaleAndOffset[i:j])
	i = j+1
	j = i
	offset = float(scaleAndOffset[i:len(scaleAndOffset)-1])
	print(scale)
	print(offset)
	val = getValue(first,second)*scale + offset
	return val



def processLine(line):
	l = line.split()
	print(l)
	signal = getName(l[1])
	val = extractValue(l[4],l[5])
	packet = [signal,val]
	logs.append(packet)


def start(filename):
	file = open(filename, 'r')
	lines = file.readlines()
	for line in lines:
		processLine(line)
	csvFilename = filename[:-4] + ".csv"
	with open(csvFilename, 'w', encoding='UTF8', newline='') as f:
	    writer = csv.writer(f)

	    # write multiple rows
	    writer.writerows(logs)



if __name__ == '__main__':
	start(sys.argv[1])
