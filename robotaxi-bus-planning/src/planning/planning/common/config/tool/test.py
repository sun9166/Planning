import os
from sys import argv
try:
    script,first = argv
except :
    print "usage: python generate_png.py [config_file]"
    os._exit(0)

print "reading file" ,first
try:
	f = open(first,"r")
	lines = f.readlines()
	f.close();
except :
    print "reading file ", first , " failed"
    os._exit(0)

# print lines
out_lines=[]
for line in lines:
	lineArr = line.strip().split()
	if len(lineArr) == 0:
		continue
	if len(lineArr) != 4:
		continue
	if lineArr[2] == '-':
		continue
	if lineArr[1] == 'Reset':
		continue
	if lineArr[1] == 'BehaviorReset':
                continue
	str = lineArr[0] + "\t->\t"+ lineArr[2] + "[label = " + lineArr[1]+"];\n"
	out_lines.append(str);
	

header="""digraph G {
   rankdir=LR;
"""
end="\n}"
file_object = open('temp.dot', 'w')
file_object.writelines(header);
file_object.writelines(out_lines);
file_object.write(end);
file_object.close();
os.system("dot -Tpng temp.dot -o test.png")

	


 
